import numpy as np,pytest
from camera_heading_observation_probe import CameraHeadingObservationProbe
from active_course_observation_probe import ActiveCourseObservationProbe

def call(p,t,i,deg=0,axis=None,ready=True):
 a=np.array([np.cos(np.deg2rad(deg)),np.sin(np.deg2rad(deg)),0.]) if axis is None else axis
 return p.choose(t,i,t,0,ready,np.ones(10,np.float32),t,a)

def test_actual_old_proxy_counterexample_reproduces_and_new_axis_keeps_turn():
 old=ActiveCourseObservationProbe();v=lambda d:.12*np.array([np.cos(np.deg2rad(d)),np.sin(np.deg2rad(d)),0.])
 old.choose(20,200,20,0,True,np.ones(10),20,v(0));old.choose(20.5,205,20.5,0,True,np.ones(10),20.5,v(0))
 np.testing.assert_equal(old.choose(20.7,205,20.5,0,True,np.ones(10),20.7,v(10.2048)),np.zeros(10))
 new=CameraHeadingObservationProbe();call(new,20,200);call(new,20.5,205)
 out=new.choose(20.7,205,20.5,0,True,np.ones(10,np.float32),20.7,np.array([np.cos(np.deg2rad(.7213)),np.sin(np.deg2rad(.7213)),0.]))
 np.testing.assert_allclose(out.reshape(5,2),np.tile([.5,-np.pi/6],(5,1)));assert not new.diagnostics['camera_heading_goal_reached']

def test_actual_axis_goal_latched_and_total_hard_exit():
 p=CameraHeadingObservationProbe();call(p,20,200);call(p,20.5,205)
 np.testing.assert_equal(call(p,21,210,11),np.zeros(10));assert p.diagnostics['camera_heading_goal_reached']
 np.testing.assert_equal(call(p,21.5,215,-1),np.zeros(10));np.testing.assert_equal(call(p,26.5,265),np.ones(10))

def test_invalid_vertical_event_axis_never_waits_or_invents_heading():
 p=CameraHeadingObservationProbe();call(p,20,200,axis=np.array([0.,0.,1.]));np.testing.assert_equal(call(p,20.5,205,axis=np.array([0.,0.,1.])),np.zeros(10))
 np.testing.assert_equal(call(p,21,210),np.zeros(10));np.testing.assert_equal(call(p,26.5,265),np.ones(10))

def test_causal_axis_timestamp_before_action_and_axis_payload():
 class Trap:
  def __array__(self,*a,**k):raise AssertionError('future payload read')
 p=CameraHeadingObservationProbe()
 with pytest.raises(ValueError,match='axis clock'):p.choose(1,10,1,0,True,Trap(),1.1,Trap())
 assert p.event_time is None

def test_wrong_velocity_vector_cannot_be_used_as_unit_camera_axis():
 p=CameraHeadingObservationProbe()
 with pytest.raises(ValueError,match='unit'):call(p,20,200,axis=np.array([.1314735,.0539788,.003525]))
 assert p.event_time is None

def test_four_second_limit_negative_heading_and_wrap():
 p=CameraHeadingObservationProbe();call(p,20,200,179);call(p,20.5,205,179);call(p,21,210,-179);assert not p.diagnostics['camera_heading_goal_reached']
 np.testing.assert_equal(call(p,24.5,245,160),np.zeros(10));assert not p.diagnostics['camera_heading_goal_reached']
