import numpy as np
from motion_measurement import xy_curvature,qualification

def test_world_xy_curvature_independent_of_robot_yaw():
    t=np.linspace(0,6,601);omega=.2;radius=2
    p=np.column_stack([radius*np.sin(omega*t),radius*(1-np.cos(omega*t))])
    v=np.column_stack([radius*omega*np.cos(omega*t),radius*omega*np.sin(omega*t)])
    k,course,valid=xy_curvature(t,p,v)
    assert valid[5:-5].all()
    np.testing.assert_allclose(k[5:-5],.5,atol=1e-5)
    kr,_,vr=xy_curvature(t,p[::-1],-v[::-1])
    np.testing.assert_allclose(kr[5:-5],-.5,atol=1e-5)

def test_zero_motion_and_stop_rebounds_are_not_curvature_or_stop():
    t=np.arange(10)*.1;p=np.zeros((10,2));v=np.zeros_like(p)
    k,c,valid=xy_curvature(t,p,v)
    assert not valid.any() and np.isnan(k).all() and np.isnan(c).all()
    q,counts=qualification(np.array([.01,.01,.06,.01,.01,.01,.01,.01,.06,.01]))
    assert np.flatnonzero(q).tolist()==[7]
    assert counts[8]==0 and counts[9]==1
