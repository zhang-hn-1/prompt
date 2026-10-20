import numpy as np
from right_temporal_observability import triangulate

def project(p,T):
 q=(p-T[:3,3])@T[:3,:3];return q[:,:2]/q[:,2:3]*584.8888888888889+[319.5,239.5]

def test_exact_calibrated_rays():
 A=np.eye(4);B=np.eye(4);B[0,3]=.2;p=np.array([[0.,0.,2.],[.1,.1,1.5]])
 w,v=triangulate(project(p,A),A,project(p,B),B)
 assert v.all();np.testing.assert_allclose(w,p,atol=1e-12)

def test_small_baseline_and_bad_correspondence_abstain():
 A=np.eye(4);B=np.eye(4);B[0,3]=.09;p=np.array([[0.,0.,2.]])
 assert not triangulate(project(p,A),A,project(p,B),B)[1].any()
 B[0,3]=.2;uv=project(p,B);uv[:,1]+=10
 assert not triangulate(project(p,A),A,uv,B)[1].any()
