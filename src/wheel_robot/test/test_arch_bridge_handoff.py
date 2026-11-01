"""Offline bridge-to-turn transition checks; no ROS Node or motion publication."""
import sys,copy,unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
if not (HERE/'test_arch_turn_gap.py').exists():
    sys.path.insert(0,str(HERE.parent/'arch_turn_brake_20261101'))
    from test_turn_gap import load_source,follower
    SOURCE=HERE.parents[2]/'shiwai_arch.py'
else:
    from test_arch_turn_gap import load_source,follower
    SOURCE=HERE.parent/'scripts/shiwai_arch.py'
import numpy as np

class BridgeHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.m,cls.init=load_source(SOURCE)
    def new(self):
        o=follower(self.m,self.init);o._cleared_bridge=False;o._cleared_at=None
        b=o._bridge;b.imu_at=b.odom_at=10.;b.baseline=(0.,0.)
        b.roll=2.8;b.pitch=4.;b.roll_rate=12.;b.yaw=7.
        o._last_command=(.20,0.);o._last_command_valid=True
        return o
    def test_done_hands_over_immediately_even_outside_old_extra_thresholds(self):
        o=self.new();o._sync_bridge(10.)
        self.assertTrue(o._cleared_bridge);self.assertEqual(o._cleared_at,10.)
        self.assertEqual(o._turn_wait_heading,7.)
        self.assertEqual(o._last_command,(.20,0.));self.assertFalse(o.sent)
        o._sync_bridge(10.1);self.assertEqual(o._cleared_at,10.)
    def test_not_done_does_not_skip_actual_bridge_confirmation(self):
        for state in ('crossing','exit_slow','exit_hold','fault'):
            o=self.new();o._bridge.state=state;o._bridge.tick=lambda now:None
            o._sync_bridge(10.)
            self.assertFalse(o._cleared_bridge,state)
    def test_stale_sensors_stop_instead_of_creeping_or_timeout_bypass(self):
        for sensor in ('imu_at','odom_at'):
            o=self.new();setattr(o._bridge,sensor,9.)
            self.assertFalse(o._bridge_settle_ready(10.))
            o._bridge_done_at=1.;o._sync_bridge(10.)
            self.assertFalse(o._cleared_bridge)
            o._bridge_settle_command(None,10.)
            self.assertEqual(o.sent[-1],(0.,0.))
    def test_near_corner_can_confirm_without_two_second_wait(self):
        o=self.new();o._sync_bridge(10.);o._update_bridge_target()
        d=o.line_detector;data_path=HERE/'fixtures/arch_turn_gap_masks.npz'
        if not data_path.exists():data_path=HERE.parent/'arch_turn_brake_20261101/recorded_masks.npz'
        data=np.load(data_path);mask=data['masks'][list(data['frame_ids']).index(141)]
        d._yellow_mask=lambda hsv,local_only=False:mask.copy()
        frame=np.zeros((480,640,3),np.uint8)
        for i in range(self.m.TURN_ENTRY_CONFIRM_FRAMES):
            with patch.object(self.m.time,'monotonic',return_value=10.+i*.07):
                r=o._detect_lane(frame)
            self.assertEqual(d.turn_entry_hold,0.)
            self.assertEqual(r.turn_entry_ready,i==self.m.TURN_ENTRY_CONFIRM_FRAMES-1)
    def test_far_corner_still_does_not_trigger_turn(self):
        o=self.new();o._sync_bridge(10.);o._update_bridge_target();d=o.line_detector
        points=np.column_stack((np.linspace(150,490,100),np.full(100,220.)))
        for _ in range(6):
            ready,row=d._curve_turn_entry([dict(measured=points)],0,0,640,480,1.)
            self.assertFalse(ready)

if __name__=='__main__':unittest.main(verbosity=2)
