import numpy as np
import measurement as m

def test_upright_pixel_and_ray_consistency():
    uv=np.array([[0.,0.],[639.,479.],[319.5,239.5],[123.,321.]])
    k=np.array([[600.,0.,319.5],[0.,580.,239.5],[0.,0.,1.]])
    newuv,newk,R=m.upright_calibration(uv,k,640)
    orig=np.column_stack((uv,np.ones(len(uv))))@np.linalg.inv(k).T
    new=np.column_stack((newuv,np.ones(len(uv))))@np.linalg.inv(newk).T
    assert np.allclose(new@R.T,orig)

def test_depth_restore_is_inverse_rotation():
    a=np.arange(480*640).reshape(480,640).astype('float32')
    assert np.array_equal(m.restore_depth(np.rot90(a)),a)

def test_point_error_accounts_for_ray_length():
    err=m.point_errors(np.array([2.,np.nan,3.]),np.array([1.,2.,3.]),np.array([[0,0,1],[0,0,1],[1,0,1.]]))
    assert err[0]==1 and np.isinf(err[1]) and err[2]==0

def test_invalid_predictions_fail_denominator():
    out=m.stats(np.array([.1,.2,np.inf]))
    assert out['n']==3 and out['within_0p3m']==2/3 and out['invalid']==1
    assert out['p90_m'] is None

def test_no_data_does_not_pass_gate():
    assert not m.subgroup_pass(m.stats([]))

def test_many_accurate_points_pass_gate():
    assert m.subgroup_pass(m.stats(np.full(150,.1)))

def test_small_good_subset_does_not_pass_gate():
    assert not m.subgroup_pass(m.stats(np.full(2,.1)))

def test_hidden_self_and_floor_not_body_obstacles():
    depth=np.array([[1.,1.,1.,1.]])
    rays=np.array([[0.,0.,1.],[0.,0.,1.],[0.,0.,1.],[0.,0.,1.]])
    centers=np.array([[0.,0.,.0],[0.,0.,.4],[0.,0.,.4],[0.,0.,.4]])
    # Rotation puts depth along world X, so four hit heights are .0/.4/.4/.4.
    R=np.array([[0,0,1],[0,1,0],[1,0,0]])
    pts=centers+rays@R.T
    valid,body=m.evaluation_masks(depth.ravel(),pts,np.array([0.,0.,.4]),np.array([False,False,True,False]))
    assert valid.tolist()==[True,True,False,True]
    assert body.tolist()==[False,True,False,True]
