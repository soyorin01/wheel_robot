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
        self.assertEqual(max(r['v'] for r in rows if r['state']=='suspect'),0.)
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
        o._bridge.imu_at=o._bridge.odom_at=10.;o._bridge.speed=.24
        self.assertTrue(o._handle_turn_gap(r,10.));self.assertEqual(o.sent[-1],(0.,-.28))
        o._bridge.speed=0.
        o._bridge.imu_at=o._bridge.odom_at=10.4
        self.assertTrue(o._handle_turn_gap(r,10.4));self.assertEqual(o.sent[-1],(0.,0.))
        o._bridge.imu_at=o._bridge.odom_at=10.5
        self.assertTrue(o._handle_turn_gap(r,10.5));self.assertEqual(o.sent[-1],(0.,0.))
        self.assertEqual(o._turn_gap_since,10.)

    def test_distance_and_sensor_stops_latch(self):
        for reason in ('distance','sensor'):
            o=self.new();self.warm(o);r=self.detect(o,self.masks[150])
            o._last_command=(.18,-.28);o._last_command_valid=True
            o._bridge.imu_at=o._bridge.odom_at=10.;o._bridge.speed=.24;o._handle_turn_gap(r,10.)
            if reason=='distance':o._bridge.distance=.161
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


    def start_gap(self):
        o=self.new();self.warm(o);r=self.detect(o,self.masks[150])
        o._last_command=(.18,-.28);o._last_command_valid=True
        o._bridge.imu_at=o._bridge.odom_at=10.;o._bridge.speed=.24
        o._handle_turn_gap(r,10.)
        return o,r

    def test_braking_keeps_right_turn_until_measured_speed_falls(self):
        o,r=self.start_gap()
        self.assertEqual(o.sent[-1],(0.,-.28))
        for t,v in [(10.1,.25),(10.3,.20),(10.5,.09)]:
            o._bridge.imu_at=o._bridge.odom_at=t;o._bridge.speed=v
            o._handle_turn_gap(r,t);self.assertEqual(o.sent[-1],(0.,-.28))
        o._bridge.imu_at=o._bridge.odom_at=10.6;o._bridge.speed=.03
        o._handle_turn_gap(r,10.6);self.assertEqual(o.sent[-1],(0.,0.))
        self.assertEqual(o._turn_gap_phase,'wait')

    def test_brake_time_yaw_distance_and_stale_bounds(self):
        for bound in ('time','yaw','distance','stale'):
            o,r=self.start_gap();now=10.81 if bound=='time' else 10.1
            o._bridge.imu_at=o._bridge.odom_at=now
            if bound=='yaw':o._bridge.yaw=-12.1
            if bound=='distance':o._bridge.distance=.161
            if bound=='stale':o._bridge.imu_at=9.
            o._turn_gap_tick(now)
            self.assertEqual(o.sent[-1],(0.,0.),bound)

    def test_scan_requires_still_current_near_edge_and_is_one_shot(self):
        for bound in ('time','yaw','distance','motion','lost','stale_camera','stale_imu'):
            o,r=self.start_gap();o._bridge.speed=0.
            o._bridge.imu_at=o._bridge.odom_at=10.1;o._handle_turn_gap(r,10.1)
            o.line_detector.turn_gap_near_ready=False
            o._bridge.imu_at=o._bridge.odom_at=10.4;o._handle_turn_gap(r,10.4)
            self.assertEqual(o.sent[-1],(0.,0.))
            o.line_detector.turn_gap_near_ready=True;o._handle_turn_gap(r,10.41)
            self.assertEqual(o.sent[-1],(0.,-.20))
            now=11.42 if bound=='time' else 10.51
            o._bridge.imu_at=o._bridge.odom_at=now;o._turn_gap_observed_at=now
            if bound=='yaw':o._bridge.yaw=-12.1
            if bound=='distance':o._bridge.distance=.041
            if bound=='motion':o._bridge.speed=.09
            if bound=='lost':o.line_detector.turn_gap_near_ready=False
            if bound=='stale_camera':o._turn_gap_observed_at=now-.21
            if bound=='stale_imu':o._bridge.imu_at=now-1.
            o._turn_gap_tick(now);self.assertEqual(o.sent[-1],(0.,0.),bound)
            o._bridge.speed=0.;o._bridge.yaw=0.;o._bridge.distance=0.
            o.line_detector.turn_gap_near_ready=True
            o._bridge.imu_at=o._bridge.odom_at=now+.1;o._handle_turn_gap(r,now+.1)
            self.assertEqual(o.sent[-1],(0.,0.),bound+' rearmed')

    def test_no_new_frame_stops_rotation_when_speed_falls(self):
        o,r=self.start_gap();o._bridge.speed=.02
        o._bridge.imu_at=o._bridge.odom_at=10.1
        o._camera_ok=True;o._frame=self.frame;o._frame_time=10.1
        o._sequence=o._last_sequence=5;o._sync_bridge=lambda now:None
        with patch.object(self.m.time,'monotonic',return_value=10.1):o._control_cycle()
        self.assertEqual(o.sent[-1],(0.,0.))

    def test_fault_or_missing_camera_never_publishes_transient_rotation(self):
        for cause in ('fault','camera'):
            o,r=self.start_gap();o.sent.clear()
            o._bridge.imu_at=o._bridge.odom_at=10.1
            o._camera_ok=cause!='camera';o._frame=self.frame;o._frame_time=10.1
            o._sequence=o._last_sequence=5;o._sync_bridge=lambda now:None
            if cause=='fault':o._bridge.state='fault'
            with patch.object(self.m.time,'monotonic',return_value=10.1):o._control_cycle()
            self.assertTrue(o.sent)
            self.assertTrue(all(c==(0.,0.) for c in o.sent))

    def test_wrong_previous_steering_or_disabled_motion_cannot_start_scan(self):
        for disabled in (True,False):
            o=self.new();self.warm(o);r=self.detect(o,self.masks[150])
            o.enable_motion=not disabled
            o._last_command=(.18,-.28 if disabled else .28);o._last_command_valid=True
            o._bridge.imu_at=o._bridge.odom_at=10.;o._bridge.speed=.24
            o._handle_turn_gap(r,10.);self.assertEqual(o.sent[-1],(0.,0.))
            self.assertEqual(o._turn_gap_phase,'stop')

    def test_depth_recovery_needs_real_support_and_three_frames(self):
        o,r=self.start_gap();d=o.line_detector
        c=d._curve_debug['boundaries'][0]
        y0=d._curve_debug['y0'];x0=d._curve_debug['x0']
        # Depth measured in ROI coordinates; near threshold is in full-image coordinates.
        c['measured']=np.column_stack((np.linspace(180,60,150),np.linspace(310-y0,459-y0,150)))
        c['horizontal']=False
        for i in range(3):
            q=copy.copy(r);q.valid=True;q.confidence=.90;q.lateral_error=.15
            d._turn_gap_check(q,True)
            self.assertEqual(q.valid,i==2)
        self.assertEqual(d.turn_gap_state,'clear')

    def test_latest_recording_does_not_accept_short_corner_as_midpoint(self):
        path=HERE/'fixtures/arch_turn_brake_masks.npz'
        if not path.exists():path=HERE/'arch_turn_brake_masks.npz'
        data=np.load(path);o=self.new();rows=[]
        for i,mask in zip(data['frame_ids'],data['masks']):
            r=self.detect(o,mask)
            if i==115:
                o.line_detector._curve_turn_entry_ready=True;o.line_detector._curve_turn_entry_frames=12
            rows.append((int(i),r.valid,r.error,o.line_detector.turn_gap_state,o.line_detector.turn_gap_near_ready))
        byid={r[0]:r for r in rows}
        self.assertTrue(any(r[3]=='joined' for r in rows if 140<=r[0]<=150))
        self.assertEqual(byid[150][3],'suspect');self.assertFalse(byid[150][1])
        self.assertFalse(any(r[1] and r[2]<0 for r in rows if 149<=r[0]<=165))
        # The actual late recording flickers: it must not be promoted into a reliable center.
        self.assertFalse(any(r[1] for r in rows if r[0]>180))
        # A stable repeat of a real near segment supports searching, not forward driving.
        mask=data['masks'][list(data['frame_ids']).index(220)]
        for _ in range(5):r=self.detect(o,mask)
        self.assertTrue(o.line_detector.turn_gap_near_ready)
        self.assertFalse(r.valid)

if __name__=='__main__':unittest.main(verbosity=2)
