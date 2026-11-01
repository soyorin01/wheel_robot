"""Pure offline regression: no ROS initialization, camera, serial or publishers.

Usage: python3 test_turn_gap.py [--source /path/to/shiwai_arch.py]
The fixture reconstructs white masks from annotated video, not raw camera HSV.
"""
import ast,copy,json,sys,types,unittest
from unittest.mock import patch
from pathlib import Path
import cv2,numpy as np

HERE=Path(__file__).resolve().parent
if '--source' in sys.argv:
    i=sys.argv.index('--source');SOURCE=Path(sys.argv[i+1]);del sys.argv[i:i+2]
else:
    SOURCE=HERE.parent/'scripts/shiwai_arch.py'
    if not SOURCE.exists():SOURCE=HERE.parents[2]/'shiwai_arch.py'

def load_source(path):
    tree=ast.parse(path.read_bytes());body=[]
    for n in tree.body:
        if isinstance(n,ast.Import) and any(a.name=='rclpy' for a in n.names):continue
        if isinstance(n,ast.ImportFrom) and (n.module or '').startswith(('rclpy','geometry_msgs','sensor_msgs','nav_msgs')):continue
        if isinstance(n,ast.If):continue
        body.append(n)
    m=types.ModuleType('turn_gap_offline');m.Node=m.Imu=m.Odometry=object
    sys.modules[m.__name__]=m
    exec(compile(ast.Module(body=body,type_ignores=[]),str(path),'exec'),m.__dict__)
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ShiWaiFollower')
    init=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
    return m,init

def follower(m,init):
    o=m.ShiWaiFollower.__new__(m.ShiWaiFollower)
    o.declare_parameter=lambda k,v:setattr(o,k,v)
    o._declare_parameters();o._apply_field_tuning()
    exclude={'cmd_vel_pub','posture_pub','_imu_sub','_odom_sub','_hdmi_size','_capture_thread','_timer'}
    for n in init.body:
        if isinstance(n,(ast.Assign,ast.AnnAssign)):
            target=n.targets[0] if isinstance(n,ast.Assign) else n.target
            if isinstance(target,ast.Attribute) and target.attr not in exclude:
                exec(compile(ast.Module(body=[n],type_ignores=[]),'<offline init>','exec'),m.__dict__,{'self':o})
    o.enable_motion=True;o._cleared_bridge=True;o._bridge.state='done';o._update_bridge_target()
    o.line_detector.turn_entry_hold=0
    o.sent=[];o.events=[]
    o._publish=lambda v,w:o.sent.append((v,w))
    o._announce=o.events.append;o._log=lambda *a:None
    o._show=lambda *a,**k:None;o._update_height=lambda now:None
    return o

class TurnGapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.m,cls.init=load_source(SOURCE)
        path=HERE/'fixtures/arch_turn_gap_masks.npz'
        if not path.exists():path=HERE/'recorded_masks.npz'
        fixture=np.load(path)
        cls.masks={int(i):mask for i,mask in zip(fixture['frame_ids'],fixture['masks'])}
        cls.frame=np.zeros((480,640,3),np.uint8)

    def new(self):return follower(self.m,self.init)

    def detect(self,o,mask):
        o.line_detector._yellow_mask=lambda hsv,local_only=False:mask.copy()
        return o.line_detector.detect(self.frame)

    def warm(self,o,end=143):
        for i in range(115,end+1):
            r=self.detect(o,self.masks[i])
            if i==115:
                o.line_detector._curve_turn_entry_ready=True
                o.line_detector._curve_turn_entry_frames=12
        return r

    def test_recorded_gap_preserves_turn_then_stops_without_reverse(self):
        o=self.new();rows=[];o._last_motion_time=20.
        for i in range(115,167):
            now=20.+(i-115)/15.;o._bridge.imu_at=o._bridge.odom_at=now
            o._bridge.distance+=max(0.,o._last_command[0])/15.
            r=self.detect(o,self.masks[i])
            if i==115:
                o.line_detector._curve_turn_entry_ready=True;o.line_detector._curve_turn_entry_frames=12
                continue
            guarded=o._handle_turn_gap(r,now)
            if not guarded:
                if r.valid:o._follow_command(r,now)
                else:o._handle_lost_line(r,now)
            v,w=o.sent[-1]
            rows.append(dict(frame=i,valid=bool(r.valid),error=r.error,v=v,w=w,state=o.line_detector.turn_gap_state))
        byid={r['frame']:r for r in rows}
        self.assertEqual(byid[144]['state'],'joined')
        self.assertGreater(byid[144]['error'],0.)
        self.assertTrue(all(r['w']<=0 for r in rows if r['frame']>=144))
        self.assertTrue(all(r['v']==0. and r['w']==0. for r in rows if r['frame']>=152))
        self.assertLessEqual(max(r['v'] for r in rows if r['state']=='suspect'),self.m.TURN_GAP_SPEED_MAX)
        self.assertTrue(all(not r['valid'] for r in rows if r['state']=='suspect'))

    def test_join_uses_only_observed_pixels_and_does_not_make_two_sides(self):
        o=self.new();self.warm(o);r=self.detect(o,self.masks[144]);d=o.line_detector._curve_debug
        self.assertEqual(o.line_detector.turn_gap_state,'joined')
        self.assertEqual(r.boundary_side,'left');self.assertEqual(len(d['boundaries']),1)
        c=d['boundaries'][0];measured=c['measured'].astype(int)
        # Measured samples remain from the two current islands, never the empty gap.
        gaps=np.diff(measured[:,0]);j=int(np.argmax(gaps))
        self.assertGreater(gaps[j],100)
        a,b=measured[j,0],measured[j+1,0]
        self.assertFalse(np.any((c['path'][:,0]>a)&(c['path'][:,0]<b)&c['support']))
        self.assertLess(r.confidence,.90)

    def test_wrong_far_fragment_is_not_joined(self):
        o=self.new();self.warm(o)
        mask=self.masks[144].copy();mask[:,350:]=0
        cv2.rectangle(mask,(450,180),(639,220),255,-1)  # Far from the previous ground boundary.
        r=self.detect(o,mask)
        self.assertEqual(o.line_detector.turn_gap_links,0)
        self.assertEqual(o.line_detector.turn_gap_state,'suspect');self.assertFalse(r.valid)

    def test_missing_line_does_not_drop_guard_or_reset_budget(self):
        o=self.new();self.warm(o);self.detect(o,self.masks[150])
        r=self.detect(o,np.zeros((480,640),np.uint8))
        self.assertEqual(o.line_detector.turn_gap_state,'suspect')
        o._last_command=(.18,-.28);o._last_command_valid=True
        o._bridge.imu_at=o._bridge.odom_at=10.
        self.assertTrue(o._handle_turn_gap(r,10.));self.assertEqual(o.sent[-1],(.10,-.28))
        o._bridge.imu_at=o._bridge.odom_at=10.4
        self.assertTrue(o._handle_turn_gap(r,10.4));self.assertEqual(o.sent[-1],(0.,0.))
        o._bridge.imu_at=o._bridge.odom_at=10.5
        self.assertTrue(o._handle_turn_gap(r,10.5));self.assertEqual(o.sent[-1],(0.,0.))
        self.assertEqual(o._turn_gap_since,10.)

    def test_distance_and_sensor_stops_latch(self):
        for reason in ('distance','sensor'):
            o=self.new();self.warm(o);r=self.detect(o,self.masks[150])
            o._last_command=(.18,-.28);o._last_command_valid=True
            o._bridge.imu_at=o._bridge.odom_at=10.;o._handle_turn_gap(r,10.)
            if reason=='distance':o._bridge.distance=.041
            else:o._bridge.imu_at=9.
            o._handle_turn_gap(r,10.1);self.assertEqual(o.sent[-1],(0.,0.))
            o._bridge.imu_at=o._bridge.odom_at=10.2;o._bridge.distance=0.
            o._handle_turn_gap(r,10.2);self.assertEqual(o.sent[-1],(0.,0.))

    def test_no_new_camera_frame_cannot_extend_hold_budget(self):
        o=self.new();self.warm(o);r=self.detect(o,self.masks[150])
        o._last_command=(.18,-.28);o._last_command_valid=True
        o._bridge.imu_at=o._bridge.odom_at=10.;o._handle_turn_gap(r,10.)
        o._bridge.imu_at=o._bridge.odom_at=10.4
        o._camera_ok=True;o._frame=self.frame;o._frame_time=10.4
        o._sequence=o._last_sequence=5;o._sync_bridge=lambda now:None
        with patch.object(self.m.time,'monotonic',return_value=10.4):o._control_cycle()
        self.assertEqual(o.sent[-1],(0.,0.));self.assertTrue(o._turn_gap_stopped)

    def test_stable_recovery_resets_derivative_and_allows_real_correction(self):
        o=self.new();self.warm(o);r=self.detect(o,self.masks[150])
        o._last_command=(.18,-.28);o._last_command_valid=True
        o._bridge.imu_at=o._bridge.odom_at=10.;o._handle_turn_gap(r,10.)
        for i in range(2):
            r=self.detect(o,self.masks[141]);self.assertFalse(r.valid)
        r=self.detect(o,self.masks[141]);self.assertTrue(r.valid)
        self.assertFalse(o._handle_turn_gap(r,10.2));self.assertIsNone(o._turn_gap_since)
        self.assertFalse(o._have_error)
        # A full, consistently observed boundary can eventually request left correction.
        # The guard is not a permanent ban on turning left.
        for i in range(3):
            q=copy.copy(r);q.lateral_error=-.3;q.error=-.2;q.valid=True;q.confidence=.95
            o.line_detector._turn_gap_check(q,True)
        self.assertTrue(q.valid)
        self.assertEqual(o.line_detector.turn_gap_state,'clear')

    def test_before_bridge_and_final_section_do_not_arm_turn_guard(self):
        for stage in ('before','after'):
            o=self.new()
            if stage=='before':o._cleared_bridge=False;o._bridge.state='idle'
            else:o._passed_sidewalk=True
            o._update_bridge_target()
            for i in (141,144,150):self.detect(o,self.masks[i])
            self.assertEqual(o.line_detector.turn_gap_state,'off')

    def test_lf_and_safe_import(self):
        self.assertNotIn(b'\r',SOURCE.read_bytes())
        self.assertNotIn('rclpy',sys.modules)

if __name__=='__main__':unittest.main(verbosity=2)
