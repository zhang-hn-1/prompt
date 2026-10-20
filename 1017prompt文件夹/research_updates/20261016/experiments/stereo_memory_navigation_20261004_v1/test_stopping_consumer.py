import numpy as np
from stopping_consumer import stop_disc_filter,StoppingConsumer


def test_measured_tail_rejects_hazard_outside_static_footprint():
    grid=np.full((41,41),-1,np.int8);grid[27,20]=1
    action=np.tile([.3,.25],5).astype(np.float32)
    final,info=stop_disc_filter(action,[0,0],grid,[-2,-2],.1)
    assert not final.any() and info['stop_veto']
    assert info['empirical_tail_m']>.14
    assert info['unknown_cells']>0 and not info['full_3d_certified']
    np.testing.assert_array_equal(action,np.tile([.3,.25],5).astype(np.float32))
    assert grid[20,20]==-1


def test_unknown_remains_explicit_risk_and_outside_hazard_does_not_veto():
    grid=np.full((41,41),-1,np.int8);grid[29,20]=1
    action=np.tile([.3,-.25],5)
    final,info=stop_disc_filter(action,[0,0],grid,[-2,-2],.1)
    np.testing.assert_array_equal(final,action.astype(np.float32))
    assert not info['stop_veto'] and info['unknown_cells']>0
    assert info['support_status']=='unknown_risk' and not info['certified_stopping_bound']


def test_stop_qualification_uses_five_current_samples_with_rebound_reset():
    consumer=StoppingConsumer()
    for speed in [.02,.02,.06,.02,.02,.02,.02]:
        assert not consumer.observe_stop(speed)
    assert consumer.low_count==4
    assert consumer.observe_stop(.02)
    assert not consumer.observe_stop(.051) and consumer.low_count==0


def test_past_prefix_only_and_zero_request_is_not_physical_stop():
    consumer=StoppingConsumer();grid=np.full((41,41),-1,np.int8)
    action=np.zeros(10)
    final,info=consumer.apply(action,[0,0],.12,grid,[-2,-2],.1)
    assert not final.any() and not info['actual_stop_qualified']
    assert info['low_speed_count']==0
    assert not info['certified_stopping_bound']


def test_live_policy_consumes_observation_once_and_changes_final_action():
    from types import SimpleNamespace
    from stopping_policy import InformationPolicy
    from training.action_protocol import load_action_protocol
    p=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
    policy=InformationPolicy(p,'visible','points',memory_mode='history')
    obs=SimpleNamespace(position_world_xy=np.zeros(2),yaw_world_rad=0.)
    policy.set_current_packet(dict(time_s=6.,points_world=np.array([[.7,0,.4]]),root_xyz=np.array([0.,0.,.4]),speed_xy=.12))
    action=policy.act(obs,np.array([3.,0.]))
    assert len(policy.records)==1 and policy._packet is None
    assert np.any(policy.records[0]['proposed_request']) and not action.any()
    assert policy.records[0]['stopping_consumer']['stop_veto']
    assert policy.records[0]['body_prior_observed_free']==0
    assert policy.records[0]['footprint_unknown']==49
    assert not policy.records[0]['stopping_consumer']['actual_stop_qualified']
