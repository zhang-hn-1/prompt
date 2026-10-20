"""Expose the frozen consumer's prior; do not silently relabel it observation."""
from types import SimpleNamespace
import numpy as np
from risk_policy import InformationPolicy
from training.action_protocol import load_action_protocol

def test_actual_risk_consumer_contains_unobserved_body_footprint_prior():
    protocol=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
    policy=InformationPolicy(protocol,'visible','surface')
    policy.set_current_packet(dict(time_s=0.,points_world=np.empty((0,3)),root_xyz=np.array([0.,0.,.4]),speed_xy=0.,ground_meshes=[]))
    policy.act(SimpleNamespace(position_world_xy=np.zeros(2),yaw_world_rad=0.),np.array([2.,0.]))
    prior=policy.belief.labels==0
    assert prior.sum()==49
    assert np.all(~np.isfinite(policy.belief.last_seen[prior]))
    assert policy.records[-1]['known_free']==49
