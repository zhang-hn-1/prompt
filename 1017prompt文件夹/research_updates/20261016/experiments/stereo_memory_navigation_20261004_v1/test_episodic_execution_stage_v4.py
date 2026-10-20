"""Independent geometry/causal prefix tests for the real offline producer."""
import importlib,json
import numpy as np
import pytest

def module():return importlib.import_module('run_episodic_execution_memory_v3')
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

@pytest.mark.parametrize('bad',['protocol','design','complete','count','missing','duplicate','outside'])
def test_preteacher_integrity_rejects_wrong_or_incomplete_seal(bad):
 m=module();pred=[dict(episode=e,frame=f) for e in m.TEST for f in range(60,201,10)]
 seal=dict(protocol_sha256='p',design_sha256='d',prediction_run_complete=True,heldout_future_states_opened=False,queries=30)
 if bad=='protocol':seal['protocol_sha256']='x'
 if bad=='design':seal['design_sha256']='x'
 if bad=='complete':seal['prediction_run_complete']=False
 if bad=='count':seal['queries']=29
 if bad=='missing':pred.pop()
 if bad=='duplicate':pred[-1]=pred[0].copy()
 if bad=='outside':pred[-1]['frame']=210
 with pytest.raises(ValueError):m.verify_prediction_seal(seal,pred,'p','d')
def test_exact_complete_prediction_seal_is_admitted():
 m=module();pred=[dict(episode=e,frame=f) for e in m.TEST for f in range(60,201,10)]
 seal=dict(protocol_sha256='p',design_sha256='d',prediction_run_complete=True,heldout_future_states_opened=False,queries=30)
 m.verify_prediction_seal(seal,pred,'p','d')
