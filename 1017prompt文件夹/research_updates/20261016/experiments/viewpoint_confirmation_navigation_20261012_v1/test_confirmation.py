import unittest
import numpy as np
from viewpoint_confirmation import ViewpointConfirmationGrid
class Trap:
 def __array__(self,*a,**kw):raise AssertionError('future payload accessed')
class Confirmation(unittest.TestCase):
 def grid(self):return ViewpointConfirmationGrid([-20,-20],10,static_scene_assumption=True)
 def record(self,f,t):return dict(frame=f,time_s=t,world=np.array([[1.,0.,.2]]))
 def view(self,t,x):return dict(time_s=t,position_world=np.array([x,0.,.5]))
 def test_close_exposures_remain_recent(self):
  g=self.grid();r=[self.record(0,0),self.record(5,.5)];v={0:self.view(0,0),5:self.view(.5,.068)};g.consume_records(r,.5,[0,0],v)
  self.assertEqual(int((g.labels==1).sum()),1);self.assertFalse(np.isfinite(g._confirmed_clock).any())
  g.consume_records([],11,[0,0],{});self.assertFalse((g.labels==1).any());self.assertTrue((g.labels==-1).all())
 def test_separated_exposures_persist(self):
  g=self.grid();g.consume_records([self.record(0,0),self.record(5,.5)],.5,[0,0],{0:self.view(0,0),5:self.view(.5,.1)})
  self.assertTrue(np.isfinite(g._confirmed_clock).any());g.consume_records([],11,[0,0],{});self.assertEqual(int((g.labels==1).sum()),1)
 def test_future_before_world(self):
  g=self.grid();r=self.record(0,2);r['world']=Trap()
  with self.assertRaises(ValueError):g.consume_records([r],1,[0,0],{0:self.view(2,0)})
 def test_changed_acquisition_rejected(self):
  g=self.grid();g.consume_records([self.record(0,0)],0,[0,0],{0:self.view(0,0)})
  with self.assertRaises(ValueError):g.consume_records([self.record(0,0)],1,[0,0],{0:self.view(0,.2)})
 def test_missing_pose_does_not_invent_confirmation(self):
  g=self.grid()
  with self.assertRaises(ValueError):g.consume_records([self.record(0,0)],0,[0,0],{})
  self.assertFalse(np.isfinite(g._confirmed_clock).any())
if __name__=='__main__':unittest.main()
