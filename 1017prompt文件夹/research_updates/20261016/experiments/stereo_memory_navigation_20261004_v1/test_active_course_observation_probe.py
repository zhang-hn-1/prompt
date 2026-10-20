import numpy as np,pytest
from active_course_observation_probe import ActiveCourseObservationProbe

def call(p,t,frame,heading=0.,speed=.12,st=None,ready=True):
 return p.choose(t,frame,t if st is None else st,0,ready,np.ones(10,np.float32),t,np.array([speed*np.cos(heading),speed*np.sin(heading),0.]))

def test_same_source_not_counted_and_real_course_ends_turn():
 p=ActiveCourseObservationProbe();np.testing.assert_equal(call(p,20,200),np.ones(10))
 np.testing.assert_equal(call(p,20.1,200,st=20),np.ones(10));a=call(p,20.5,205);np.testing.assert_allclose(a.reshape(5,2),np.tile([.5,-np.pi/6],(5,1)))
 np.testing.assert_equal(call(p,21,210,heading=np.deg2rad(11)),np.zeros(10));assert p.diagnostics['course_goal_reached']
 np.testing.assert_equal(call(p,21.5,215,heading=0),np.zeros(10))

def test_four_second_turn_deadline_six_second_exit_no_retrigger():
 p=ActiveCourseObservationProbe();call(p,20,200);call(p,20.5,205)
 np.testing.assert_equal(call(p,24.5,245),np.zeros(10));np.testing.assert_equal(call(p,26.5,265),np.ones(10));np.testing.assert_equal(call(p,27,270),np.ones(10))

def test_low_event_velocity_never_invents_course_or_waits_forever():
 p=ActiveCourseObservationProbe();call(p,20,200,speed=0);np.testing.assert_equal(call(p,20.5,205,speed=0),np.zeros(10))
 np.testing.assert_equal(call(p,21,210,speed=.2),np.zeros(10));assert p.diagnostics['turn_not_exercised_invalid_event_course']
 np.testing.assert_equal(call(p,26.5,265),np.ones(10))

def test_future_velocity_rejected_before_any_payload_or_mutation():
 class Bomb:
  def __array__(self,*a,**k):raise AssertionError('future payload read')
 p=ActiveCourseObservationProbe()
 with pytest.raises(ValueError,match='velocity clock'):p.choose(1,10,1,0,True,Bomb(),1.1,Bomb())
 assert p.event_time is None

def test_negative_change_and_invalid_current_speed_do_not_hit_goal():
 p=ActiveCourseObservationProbe();call(p,20,200,heading=np.deg2rad(179));call(p,20.5,205,heading=np.deg2rad(179))
 call(p,21,210,heading=np.deg2rad(-179));assert not p.diagnostics['course_goal_reached']
 call(p,21.5,215,heading=np.deg2rad(-169),speed=.01);assert not p.diagnostics['course_goal_reached']
 call(p,22,220,heading=np.deg2rad(160));assert not p.diagnostics['course_goal_reached']


def test_current_input_shape_rejected_before_state_mutation():
 p=ActiveCourseObservationProbe()
 with pytest.raises(ValueError):p.choose(1,10,1,0,True,np.ones(10),1,np.zeros(2))
 assert p.event_time is None
