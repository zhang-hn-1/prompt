import unittest,numpy as np
from persistent_hypothesis_context_v1 import PersistentHypothesisContext

def birth():return dict(time_s=0.,frame=0,raw_world_voxel_centres=np.array([[0.,0.,-1.]]),model_hypothesis=True,measured_geometry=False,certified_free=False)
def args(t=0.):return dict(time_s=t,images={e:np.ones((480,640,3),np.uint8)*100 for e in ['left','right']},masks={e:np.zeros((480,640),bool) for e in ['left','right']},poses={e:np.eye(4) for e in ['left','right']})
class Tests(unittest.TestCase):
 def test_projection_is_necessary_only_and_birth_immutable(self):
  b=birth();m=PersistentHypothesisContext(b,'a'*64,0.);b['raw_world_voxel_centres'][0,2]=-5;r=m.inspect(args(),0.);self.assertTrue(r['model_active']);self.assertEqual(r['eye_reason']['left'][0],'nonblack_projection_only');self.assertFalse(r['observed_free']);self.assertFalse(r['measured_geometry']);self.assertEqual(r['hypothesis_sha256'],'a'*64)
 def test_missing_black_or_CAD_does_not_clear_or_refresh(self):
  m=PersistentHypothesisContext(birth(),'a'*64,0.);a=args(10.);a['images']['left'][:]=0;a['masks']['right'][:]=True;r=m.inspect(a,10.);self.assertTrue(r['model_active']);self.assertEqual(r['birth_time_s'],0.);self.assertEqual(r['eye_reason']['left'][0],'black_RGB_unknown');self.assertEqual(r['eye_reason']['right'][0],'CAD_unknown');self.assertFalse(m.inspect(args(60.1),60.1)['model_active'])
 def test_outside_frustum_is_not_absence(self):
  b=birth();b['raw_world_voxel_centres'][0]=[10,0,-1];m=PersistentHypothesisContext(b,'a'*64,0.);r=m.inspect(args(),0.);self.assertEqual(r['eye_reason']['left'][0],'outside_projectable_frustum');self.assertTrue(r['model_active'])
 def test_future_before_payload(self):
  class Bomb(dict):
   def __getitem__(self,k):
    if k in ['images','raw_world_voxel_centres']:raise AssertionError('future payload read')
    return super().__getitem__(k)
  with self.assertRaises(ValueError):PersistentHypothesisContext(Bomb(time_s=1.),'a'*64,0.)
  m=PersistentHypothesisContext(birth(),'a'*64,0.)
  with self.assertRaises(ValueError):m.inspect(Bomb(time_s=1.),0.)
 def test_pose_validation_and_snapshot_copy(self):
  m=PersistentHypothesisContext(birth(),'a'*64,0.);a=args();a['poses']['left'][0,0]=2
  with self.assertRaises(ValueError):m.inspect(a,0.)
  r=m.inspect(args(),0.);r['eye_reason']['left'][0]='modified';self.assertEqual(m.inspect(args(),0.)['eye_reason']['left'][0],'nonblack_projection_only')
if __name__=='__main__':unittest.main()
