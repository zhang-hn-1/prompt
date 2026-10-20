import numpy as np
from ground_contact_hypothesis import predict

def test_plane_intersection_stays_hypothesis():
 cls=np.zeros((480,640),bool);cls[200:251,320]=True;mask=np.zeros_like(cls);T=np.eye(4);T[:3,:3]=np.diag([1,-1,-1]);T[2,3]=1
 p=predict(cls,mask,T)
 assert p['label']=='GROUNDED_CONTACT_MODEL_HYPOTHESIS' and not p['certified_free']
 assert p['uv'].tolist()==[[320,250]];np.testing.assert_allclose(p['world'][:,2],0,atol=1e-12)

def test_cad_occluded_bottom_and_frame_cut_are_unknown():
 cls=np.zeros((480,640),bool);cls[200:251,320]=True;cls[300:,321]=True;mask=np.zeros_like(cls);mask[252,320]=True;T=np.eye(4);T[:3,:3]=np.diag([1,-1,-1]);T[2,3]=1
 assert len(predict(cls,mask,T)['world'])==0
