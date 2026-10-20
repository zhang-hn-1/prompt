import numpy as np,pytest
from partial_cue_source_v2 import inflate_xy,PartialCueSetSource

def test_disk_uncertainty_preserves_centre_and_radius_bound():
 xy=np.array([[0.,0.]]);out=inflate_xy(xy,.2)
 assert len(out)==13 and np.linalg.norm(out,axis=1).max()<=.2+1e-12 and (out==0).all(1).any()
 with pytest.raises(ValueError):inflate_xy(xy,.3)

def test_zero_radius_and_empty_source():
 x=np.array([[.1,.2],[.1,.2]]);np.testing.assert_equal(inflate_xy(x,0.),[[.1,.2]])
 assert inflate_xy(np.empty((0,2)),.2).shape==(0,2)

def test_future_source_rejected_before_any_payload():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return 1.
 with pytest.raises(ValueError):PartialCueSetSource(.2).update(Bomb(),None,None,None,False,0.,0,None,None,None)

def test_model_hypothesis_clock_birth_and_no_false_observation():
 model=PartialCueSetSource(.2);root=np.array([-2.2,0.]);goal=np.array([2.2,0.]);T=np.eye(4);T[:3,:3]=np.array([[0,0,-1],[0,1,0],[1,0,0]])
 T[:3,3]=[-2.,.3,.45];R=T.copy();R[1,3]=-.3
 poses=dict(left_camera_usd_world=T,right_camera_usd_world=R);mask=dict(left=np.zeros((480,640),bool),right=np.zeros((480,640),bool));rgb=dict(left_rgb=np.full((480,640,3),50,np.uint8),right_rgb=np.full((480,640,3),50,np.uint8))
 def p(i):
  w=np.tile([[0.,0.,.5],[.1,0.,.5]],(50,1));return dict(time_s=i*.1,frame=i,frame_width=640,world=w,query_pixel_id=np.full(100,240*640+320),partner_pixel_id=np.full(100,240*640+320))
 a=model.update(p(0),rgb,poses,mask,False,0.,0,root,goal,root)
 assert not len(a['possible_xy']) and a['pending_seed']
 b=model.update(p(5),rgb,poses,mask,True,.5,5,root,goal,root)
 assert len(b['possible_xy']) and b['model_hypothesis'] and not b['measured_geometry'] and not b['certified_free']
 assert b['source_seed_time_s']==.5
 saved=model.birth_packet['possible_xy'].copy();rgb={k:np.zeros_like(v) for k,v in rgb.items()}
 for i in [10,15,20,25]:c=model.update(p(i),rgb,poses,mask,False,i*.1,i,root,goal,root)
 assert not len(c['possible_xy']) and c['source_seed_time_s']==.5 and c['time_s']==2.5
 np.testing.assert_equal(model.birth_packet['possible_xy'],saved)
 assert model.birth_packet['time_s']==.5
