"""Hand-derived projection, actual textured-plane geometry and causal safeguards."""
import numpy as np,pytest
from temporal_ray_sweep import project_depth,TemporalRaySweep
K=np.array([[40.,0,31.5],[0,40.,23.5],[0,0,1.]])
def pose(x=0):
 T=np.eye(4);T[:3,:3]=np.diag([1,-1,-1]);T[0,3]=x;return T
def fixture():
 rng=np.random.default_rng(10);tex=rng.integers(0,255,(48,96),dtype=np.uint8);out=[]
 for t,x,shift in [(2.,0.,0),(1.,.2,8),(0.,.4,16)]:
  gray=tex[:,shift:shift+64];out.append(dict(time_s=t,rgb=np.repeat(gray[:,:,None],3,axis=2),pose=pose(x),mask=np.zeros((48,64),bool)))
 return out
# Catch wrong USD handedness/baseline direction or ray depth units.
def test_literal_projection_of_one_metre_plane():
 uv,z=project_depth(np.array([[31.5,23.5],[51.5,23.5]]),1.,pose(),pose(.2),K)
 assert np.allclose(uv,[[23.5,23.5],[43.5,23.5]],atol=1e-10);assert np.allclose(z,[1,1])
# Catch biased depth/ray intersection without labels or rendered simulation.
def test_textured_plane_depth_is_one_metre():
 current,a,b=fixture();out=TemporalRaySweep(K).predict(current,[a,b],2.)
 valid=np.isfinite(out['depth_m']);assert valid.sum()>100
 assert np.quantile(np.abs(out['depth_m'][valid]-1.),.9)<=.04
# Catch fake geometry from low contrast or body-mask padding.
def test_textureless_or_masked_observation_creates_no_depth():
 current,a,b=fixture()
 for mode in ['flat','masked']:
  packets=[dict(p) for p in [current,a,b]]
  for p in packets:
   if mode=='flat':p['rgb']=np.full_like(p['rgb'],100)
   else:p['mask']=np.ones_like(p['mask'])
  assert not np.isfinite(TemporalRaySweep(K).predict(packets[0],packets[1:],2.)['depth_m']).any()
# Catch access to future image/pose data before rejection.
def test_future_payload_is_never_read():
 current,a,b=fixture()
 class Trap:
  def __getitem__(self,k):
   if k=='time_s':return 3.
   raise AssertionError('future payload read')
 with pytest.raises(ValueError):TemporalRaySweep(K).predict(current,[a,Trap()],2.)
# Catch pure-rotation/stationary updates called metric depth.
def test_stationary_camera_pair_rejects_depth():
 current,a,b=fixture();a['pose']=pose();b['pose']=pose()
 assert not np.isfinite(TemporalRaySweep(K).predict(current,[a,b],2.)['depth_m']).any()
