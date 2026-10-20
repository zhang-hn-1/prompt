"""Independent geometry/causal prefix tests for the real offline producer."""
import importlib,json
import numpy as np
import pytest

def module():return importlib.import_module('run_episodic_execution_memory_v2')
def fixture(tmp_path):
 (tmp_path/'history').mkdir();proprio=np.zeros((5,17));quats=np.tile([1.,0.,0.,0.],(5,1));np.savez(tmp_path/'history/frame_010.npz',proprio_history=proprio,action_history=np.zeros((5,10)),quaternion_world_wxyz_history=quats)
 return [dict(time_s=f*.1,request=[.5,0.]*5) for f in range(20)]
def test_proposal_and_prefix_features_invariant_to_future_realized_requests(tmp_path):
 m=module();rows=fixture(tmp_path);f,p,_=m.feature_at(tmp_path,10,rows)
 for i in range(11,20):rows[i]['request']=[-.3,.5]*5
 g,q,_=m.feature_at(tmp_path,10,rows);np.testing.assert_array_equal(f,g);np.testing.assert_array_equal(p.commands,q.commands);assert not m.honored(rows,10,p.commands)
def test_future_targets_are_body_relative_and_separate_from_current_features():
 m=module();q=[np.sqrt(.5),0,0,np.sqrt(.5)];states=[dict(time_s=i*.1,position=[2.,3.+i*.02,0.],quaternion=q) for i in range(6)]
 want=np.array([[i*.02,0.,0.] for i in range(1,6)]);np.testing.assert_allclose(m.target_at(states,0),want,atol=1e-12)
 states[-1]['time_s']+=.02
 with pytest.raises(ValueError):m.target_at(states,0)
def test_nonuniform_command_blocks_cannot_be_a_hold_proposal():
 m=module();r=dict(request=[.5,0.]*5);r['request'][-1]=.2
 with pytest.raises(ValueError):m.pair(r)
def test_missing_horizon_is_not_zero_error():
 m=module();states=[dict(time_s=0.,position=[0,0,0],quaternion=[1,0,0,0])];assert m.target_at(states,0) is None
def test_actual_past_capture_clock_mismatch_is_rejected(tmp_path):
 m=module();rows=fixture(tmp_path);rows[7]['time_s']+=.02
 with pytest.raises(ValueError):m.feature_at(tmp_path,10,rows)
