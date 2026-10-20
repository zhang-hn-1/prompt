import unittest
import numpy as np
from bearing_observability_v1 import build_context
from uncertain_bearing_memory_v1 import UncertainBearingMemory

def packet(t=1.,frame=10):
    a=np.eye(4);a[:3,:3]=np.diag([1.,-1.,-1.])
    k=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
    return build_context([[319.5,239.5]],a,k,t,frame)

class TestUncertainBearingMemory(unittest.TestCase):
    def test_current_packet_retains_observation_time_not_free_geometry(self):
        m=UncertainBearingMemory();m.update(packet(),1.,0.);s=m.snapshot(1.)
        self.assertEqual(len(s),1);self.assertEqual(s[0]['capture_time_s'],1.)
        self.assertFalse(s[0]['observed_depth']);self.assertFalse(s[0]['certified_free'])
        self.assertEqual(s[0]['role'],'uncertain_2D_bearing_history')
    def test_future_packet_rejected_without_state_change(self):
        m=UncertainBearingMemory();m.update(packet(),1.,0.)
        with self.assertRaises(ValueError):m.update(packet(3.,30),2.,0.)
        self.assertEqual(len(m.snapshot(1.)),1)
    def test_duplicate_epoch_cannot_create_independent_observation(self):
        m=UncertainBearingMemory();m.update(packet(),1.,0.);m.update(packet(),2.,0.)
        self.assertEqual(len(m.snapshot(2.)),1)
        self.assertEqual(m.snapshot(2.)[0]['capture_time_s'],1.)
    def test_expired_packet_not_refreshed_by_replay(self):
        m=UncertainBearingMemory();m.update(packet(),1.,0.);m.update(packet(),12.,0.)
        self.assertEqual(m.snapshot(12.),[])
    def test_pose_uncertainty_preserves_only_image_bearing(self):
        m=UncertainBearingMemory();m.update(packet(),1.,.1);s=m.snapshot(1.)
        self.assertFalse(s[0]['world_ray_available']);self.assertIsNone(s[0]['world_ray_directions'])
    def test_snapshot_cannot_read_future_update(self):
        m=UncertainBearingMemory();m.update(packet(),3.,0.)
        with self.assertRaises(ValueError):m.snapshot(2.)
    def test_inputs_and_snapshot_copies_do_not_change_memory(self):
        m=UncertainBearingMemory();p=packet();m.update(p,1.,0.);p['xy'][0]=0
        a=m.snapshot(1.);a[0]['query_xy'][0][0]=0
        self.assertEqual(m.snapshot(1.)[0]['query_xy'][0],[319.5,239.5])
    def test_depth_claim_cannot_upgrade_uncertain_packet(self):
        m=UncertainBearingMemory();p=packet();p['observed_depth']=True
        with self.assertRaises(ValueError):m.update(p,1.,0.)

if __name__=='__main__':unittest.main()
