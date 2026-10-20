import pytest,numpy as np
from render_only_snapshots_v1 import record_render_only

def test_pure_render_capture_preserves_state_and_clock():
 calls=[];state=np.array([1.,2.]);data=record_render_only(lambda:calls.append('render'),lambda:(0.,1.),lambda:[state.copy()],lambda:{'RGB':np.zeros((2,2,3),np.uint8)},2);assert len(data)==len(calls)==2

def test_physics_or_clock_change_rejected_before_capture():
 state=np.array([0.]);captured=[]
 def step():state[0]+=1
 with pytest.raises(AssertionError):record_render_only(step,lambda:0.,lambda:[state.copy()],lambda:captured.append(1),2)
 assert not captured

def test_bound_rejected_before_render():
 with pytest.raises(ValueError):record_render_only(lambda:1,lambda:0.,lambda:[],lambda:{},25)
