import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from temporal_dense_rectification_v1 import rectification, normalized_projection, reconstruct_world

def pose(origin,angle=0):
 T=np.eye(4);T[:3,3]=origin;T[:3,:3]=Rotation.from_euler('y',angle).as_matrix()@np.diag([1.,-1.,-1.]);return T
@pytest.mark.parametrize('baseline',[(.3,0,0),(0,.3,0),(-.3,0,0)])
def test_rectified_disparity_roundtrip_all_axis_signs(baseline):
 A=pose([0,0,0]);B=pose(baseline);r=rectification(A,B)
 world=np.array([[.1,.2,2.],[-.2,-.1,1.5]])
 left=normalized_projection(r,A,world,0);right=normalized_projection(r,B,world,1)
 np.testing.assert_allclose(left[:,1],right[:,1],atol=1e-7)
 d=left[:,0]-right[:,0];assert (d>0).all()
 np.testing.assert_allclose(reconstruct_world(r,left,d),world,atol=1e-7)

def test_relative_rotation_preserved_in_geometry():
 A=pose([0,0,0],.03);B=pose([.3,.05,.02],-.02);r=rectification(A,B)
 world=np.array([[.1,.1,2.],[-.1,.15,1.6]])
 a=normalized_projection(r,A,world,0);b=normalized_projection(r,B,world,1)
 np.testing.assert_allclose(a[:,1],b[:,1],atol=1e-6)
 np.testing.assert_allclose(reconstruct_world(r,a,a[:,0]-b[:,0]),world,atol=1e-6)

def test_zero_baseline_rejected():
 with pytest.raises(ValueError):rectification(pose([0,0,0]),pose([0,0,0]))
