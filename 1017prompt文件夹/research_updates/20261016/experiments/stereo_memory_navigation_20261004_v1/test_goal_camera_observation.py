import numpy as np
import pytest
from goal_camera_observation import GoalCameraObservation

def step(c,t,frame,axis=None,xy=None,count=0):
 return c.request(t,frame,frame*.1,count,np.array([np.cos(.35),np.sin(.35),0]) if axis is None else np.array(axis),t,np.array([0.,0.]) if xy is None else np.array(xy),np.array([4.4,0.]),np.zeros(2))

def test_two_sources_then_current_camera_axis_crossing_stops_and_never_retriggers():
 c=GoalCameraObservation(np.zeros(2));assert np.array_equal(step(c,0.,0),[0,0]);r=step(c,.5,5);assert r[0]==.5 and np.isclose(r[1],np.pi/6)
 r=step(c,.6,5,[1,-.01,0]);assert r[0]==0 and np.isclose(r[1],np.pi/6)
 assert step(c,1.,10)[0]==0

def test_displacement_and_deadline_end_turn_without_cyclic_wait():
 c=GoalCameraObservation(np.zeros(2));step(c,0.,0);step(c,.5,5);assert step(c,.6,5,xy=[.5,0])[0]==0
 d=GoalCameraObservation(np.zeros(2));step(d,0.,0);step(d,.5,5);assert step(d,4.5,45)[0]==0

def test_future_clock_rejected_before_axis_payload_and_repeated_source_not_counted():
 c=GoalCameraObservation(np.zeros(2))
 with pytest.raises(ValueError):c.request(0.,5,1.,0,None,1.,None,None,None)
 step(c,0.,0);assert step(c,.1,0)[0]==0

def test_degenerate_camera_axis_never_forces_turn():
 c=GoalCameraObservation(np.zeros(2));step(c,0.,0);assert step(c,.5,5,[0,0,1])[0]==0;assert c.phase=='inapplicable'


def test_capture_clock_comparison_rejects_misdated_native_before_orientation_payload():
 from goal_camera_observation import current_observer_inputs
 with pytest.raises(ValueError):current_observer_inputs(.1,{'time_s':.2},None,None,None,None)
 native=dict(time_s=.1,available=True,rotation_world=np.eye(3).tolist(),position_world=[0,0,.4])
 source=dict(frame=0,time_s=0.,points=0)
 result=current_observer_inputs(.1,native,source,np.eye(4),[4.4,0],[0,0])
 assert result['time_s']==.1 and result['axis_time']==.1 and result['source_time_s']==0.
