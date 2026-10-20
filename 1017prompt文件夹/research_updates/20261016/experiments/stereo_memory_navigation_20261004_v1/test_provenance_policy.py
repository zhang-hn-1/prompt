"""Catch bypassed provenance consumer and changed settle/risk/latch behavior."""
from types import SimpleNamespace
import numpy as np
from provenance_policy import InformationPolicy
from training.action_protocol import load_action_protocol


def test_actual_online_consumer_keeps_unknown_and_caps_risk_action():
    protocol=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
    policy=InformationPolicy(protocol,'visible','points',memory_mode='history')
    obs=SimpleNamespace(position_world_xy=np.zeros(2),yaw_world_rad=0.)
    policy.set_current_packet(dict(time_s=5.9,points_world=np.empty((0,3)),root_xyz=np.array([0.,0.,.4]),speed_xy=0.))
    assert not policy.act(obs,np.array([3.,0.])).any()
    policy.set_current_packet(dict(time_s=6.,points_world=np.empty((0,3)),root_xyz=np.array([0.,0.,.4]),speed_xy=0.))
    action=policy.act(obs,np.array([3.,0.]))
    assert abs(float(action[0])-.3)<1e-7
    assert np.all(policy.belief.labels==-1)
    assert policy.records[-1]['known_free']==0
    assert policy.records[-1]['body_prior_observed_free']==0
    assert policy.records[-1]['footprint_unknown']==49
    assert policy.records[-1]['full_3d_certified'] is False
    assert policy.latch.direction==1
