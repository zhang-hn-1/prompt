import unittest
import numpy as np
from ground_ambiguity import qualify_points,ground_partners,qualify_prediction
class Trap:
 def __array__(self,*a,**kw):raise AssertionError('future pose payload')
class GroundAmbiguity(unittest.TestCase):
 def data(self):
  f=584.8888888888889;cx=319.5;cy=239.5;y,x=np.indices((480,640));wx=(x-cx)/f
  def image(shift):
   wy=shift-(y-cy)/f;v=np.clip(128+40*np.sin(wx*30)+45*np.sin(wy*70)+25*np.cos((wx+wy)*50),0,255).astype(np.uint8);return np.repeat(v[:,:,None],3,axis=2)
  l=np.eye(4);l[:3,3]=[0,0,1];r=l.copy();r[1,3]=-.1
  return image(0),image(-.1),dict(time_s=0,left_camera_usd_world=l,right_camera_usd_world=r),{'left':np.zeros((480,640),bool),'right':np.zeros((480,640),bool)}
 def test_plane_geometry(self):
  l,r,p,m=self.data();q=np.array([[300.,300.]])
  v,valid=ground_partners(q,p,0);self.assertTrue(valid[0]);np.testing.assert_allclose(v,q+[0,-58.48888888888889],atol=1e-8)
 def test_wrong_floor_depth_becomes_ambiguous(self):
  l,r,p,m=self.data();q=np.array([[300.,300.],[330.,350.]])
  v=qualify_points(l,r,q,np.full(2,584.8888888888889*.1/.7),np.full(2,.7),p,0,m)
  self.assertTrue(v['ambiguous'].all());self.assertTrue((v['floor_ncc']>.99).all())
 def test_current_floor_depth_not_rejected(self):
  l,r,p,m=self.data();v=qualify_points(l,r,np.array([[300.,300.]]),np.array([58.48888888888889]),np.array([1.]),p,0,m);self.assertFalse(v['ambiguous'][0])
 def test_masked_rival_is_unknown(self):
  l,r,p,m=self.data();m['right'][:]=True;v=qualify_points(l,r,np.array([[300.,300.]]),np.array([83.555555]),np.array([.7]),p,0,m);self.assertFalse(v['ambiguous'][0])
 def test_future_before_pose_payload(self):
  l,r,p,m=self.data();p.update(time_s=1,left_camera_usd_world=Trap())
  with self.assertRaises(ValueError):ground_partners(np.array([[300.,300.]]),p,0)
 def test_prediction_is_causal_and_immutable(self):
  l,r,p,m=self.data();z=np.full((480,640),np.nan,np.float32);z[300,300]=.7;c=np.zeros(z.shape,bool);c[300,300]=True
  d=np.full(z.shape,584.8888888888889*.1/.7);pred=dict(depth_m=z,confidence=c,disparity_px=np.rot90(d),time_s=0)
  out=qualify_prediction(pred,l,r,p,0,m);self.assertFalse(out['confidence'][300,300]);self.assertTrue(pred['confidence'][300,300]);self.assertEqual(out['ground_ambiguity_count'],1)
  pred.update(time_s=1,depth_m=Trap())
  with self.assertRaises(ValueError):qualify_prediction(pred,l,r,p,0,m)
if __name__=='__main__':unittest.main()
