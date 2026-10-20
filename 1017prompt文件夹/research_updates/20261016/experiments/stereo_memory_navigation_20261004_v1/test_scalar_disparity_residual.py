import numpy as np,pytest
from scalar_disparity_residual import corrected_world

def test_frozen_pixel_bias_physical_z_and_identity_geometry():
 T=np.eye(4);T[:3,:3]=np.diag([1,-1,-1]);w,valid=corrected_world(np.array([319.5]),np.array([239.5]),np.array([150.]),5.,T)
 assert valid.tolist()==[True] and np.isclose(w[0,2],584.8888888888889*.6/155)
 w0,v0=corrected_world(np.array([319.5]),np.array([239.5]),np.array([200.]),0.,T)
 assert np.isclose(w0[0,2],584.8888888888889*.6/200)

def test_invalid_outputs_keep_denominator_and_nonfinite_model_rejected():
 T=np.eye(4);w,v=corrected_world(np.array([1.,2.]),np.array([1.,2.]),np.array([1.,-2.]),0.,T)
 assert len(w)==2 and v.tolist()==[False,False]
 with pytest.raises(ValueError):corrected_world(np.array([1.]),np.array([1.]),np.array([100.]),np.nan,T)

def test_off_axis_rotated_translated_pose_mixed_valid_and_boundaries():
 T=np.eye(4);T[:3,:3]=np.array([[0,-1,0],[1,0,0],[0,0,1]]);T[:3,3]=[1,2,3]
 x=np.array([400.,319.5,319.5,319.5]);y=np.array([100.,239.5,239.5,239.5]);z=np.array([2.,.5,3.,4.]);d=584.8888888888889*.6/z
 w,v=corrected_world(x,y,d,0.,T);assert v.tolist()==[True,True,True,False]
 expected=T[:3,3]+(np.column_stack([(x[:3]-319.5)/584.8888888888889,(y[:3]-239.5)/584.8888888888889,np.ones(3)])*z[:3,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 np.testing.assert_allclose(w[:3],expected);assert np.isnan(w[3]).all()
