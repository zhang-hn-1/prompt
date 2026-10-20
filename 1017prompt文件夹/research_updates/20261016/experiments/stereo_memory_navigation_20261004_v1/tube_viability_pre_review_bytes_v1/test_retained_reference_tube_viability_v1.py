from types import SimpleNamespace
import numpy as np
from retained_reference_tube_viability_v1 import tube_retention_screen,with_tube_viability_after_measured_reversal

def grid():
 g=SimpleNamespace(origin=np.array([-4.,-4.]),resolution=.1,labels=np.full((80,80),-1,np.int8),time=3.)
 g.labels[40,40]=1
 return g

def test_new_observed_risk_in_future_tube_cannot_be_retained_with_viable_start():
 g=grid();before=g.labels.copy();guard=dict(bridge_xy=[[-2,2],[-2,2]],suffix_xy=[[-2,2],[-.8,2],[-.8,0]])
 q=tube_retention_screen(g,guard);assert not q['pass'] and q['minimum_raster_clearance_m']<q['existing_planning_clearance_m'];assert q['existing_planning_clearance_m']>.87
 np.testing.assert_array_equal(g.labels,before);assert g.time==3.
 good=dict(bridge_xy=[[-2,2],[-2,2]],suffix_xy=[[-2,2],[-1.,2],[-1.,0]])
 assert tube_retention_screen(g,good)['pass']

class Fake:
 def __init__(self):self.reset(None)
 def reset(self,context):self.tracker=SimpleNamespace(_active_drive_direction='forward');self.records=[]
 def act(self,observation,goal):self.tracker._active_drive_direction=observation.next_direction;self.records.append({});return np.zeros(10)

def observation(main,direction):
 a=np.zeros((5,17));a[-1,12]=main;return SimpleNamespace(proprio_history=a,next_direction=direction)

def test_direction_latch_alone_does_not_arm_and_measured_reversal_arms_after_event():
 c=with_tube_viability_after_measured_reversal(Fake)();c.act(observation(0.,'reverse'),None);c.act(observation(0.,'forward'),None);assert not c.tube_viability_armed
 c.act(observation(-.2,'reverse'),None);assert not c.tube_viability_armed
 c.act(observation(-.2,'reverse'),None);assert not c.tube_viability_armed
 c.act(observation(0.,'forward'),None);assert c.tube_viability_armed and c.records[-1]['tube_viability_event_armed']
 c.reset(None);assert not c.tube_viability_armed and not c.measured_reverse_seen
