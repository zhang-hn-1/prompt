import numpy as np
import pytest
from qualified_start_v2 import policy_class
from benchmark.policies import PolicyObservation,OraclePathPolicy
from scripts.run_pointgoal_benchmark import load_action_protocol
from depth_local_curve import P

def observation():
 return PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),np.zeros((5,17),np.float32),np.zeros((5,10),np.float32),np.zeros(2),0.,quaternion_world_wxyz_history=np.tile([1.,0.,0.,0.],(5,1)))

def packet(p,t):
 p.set_source_packet(dict(frame=0,time_s=t,world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int)))
 p.set_current_packet(dict(frame=0,time_s=t,points_world=np.empty((0,3)),root_xyz=np.zeros(3)))

def test_actual_complete_factory_first_plan_and_tracker_once(monkeypatch):
 protocol=load_action_protocol(P/'configs/data_contract.yaml');original=OraclePathPolicy.act;calls=[]
 def counted(self,*a,**k):
  calls.append(self);return original(self,*a,**k)
 monkeypatch.setattr(OraclePathPolicy,'act',counted)
 out=[];policies=[]
 for enabled in (False,True):
  p=policy_class(enabled)(protocol,memory_mode='history');packet(p,.4)
  p.qualify_initial_start(.4);out.append(p.act(observation(),np.array([2.,0.])));policies.append(p)
 assert np.all(out[0]==0) and np.any(out[1][::2]!=0)
 assert policies[0].plan is None and policies[1].plan is not None
 assert len(calls)==1 and calls[0] is policies[1].tracker
 assert all(len(p.records)==1 for p in policies)
 assert [c.__name__ for c in type(policies[0]).__mro__]==[c.__name__ for c in type(policies[1]).__mro__]
 assert all(sum(c.__name__=='CurrentForwardIntent' for c in type(p).__mro__)==1 for p in policies)

def test_future_qualification_fails_before_map_update():
 p=policy_class(True)(load_action_protocol(P/'configs/data_contract.yaml'),memory_mode='history');packet(p,.4);p.qualify_initial_start(.5)
 with pytest.raises(ValueError,match='qualification'):p.act(observation(),np.array([2.,0.]))
 assert p.grid is None

def test_reset_clears_latched_qualification():
 p=policy_class(True)(load_action_protocol(P/'configs/data_contract.yaml'),memory_mode='history');p.qualify_initial_start(.4);p.reset(None)
 assert p.initial_start_qualified_time_s is None
