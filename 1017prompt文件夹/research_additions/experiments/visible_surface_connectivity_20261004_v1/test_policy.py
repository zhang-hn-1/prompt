import numpy as np
from training.action_protocol import ActionProtocol
from benchmark.policies import PolicyObservation
from benchmark.policies import PROPRIO_DIM
import information_policy as m
P=ActionProtocol('ball_action_v2_aux30',(-2.,-.5235988),(2.,.5235988),5)

def observation():
    return PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),np.zeros((1,PROPRIO_DIM),np.float32),np.zeros((1,10),np.float32),np.array([0.,0.]),0.,np.array([[1,0,0,0]],np.float32))

def test_missing_current_packet_is_rejected():
    p=m.InformationPolicy(P,'visible')
    p.reset(None)
    import pytest
    with pytest.raises(RuntimeError):p.act(observation(),np.array([2.,0.]))

def test_identical_maps_and_state_produce_identical_actions():
    a=m.InformationPolicy(P,'map');b=m.InformationPolicy(P,'visible');a.reset(None);b.reset(None)
    for p in (a,b):
        p.belief=m.BeliefGrid([-5.,-5.],shape=(100,100));p.belief.labels[:]=0
        p.set_current_packet(dict(time_s=0.,points_world=np.empty((0,3))))
    assert np.array_equal(a.act(observation(),np.array([2.,0.])),b.act(observation(),np.array([2.,0.])))

def test_unknown_route_requests_stop_and_reports_reason():
    p=m.InformationPolicy(P,'visible');p.reset(None);p.set_current_packet(dict(time_s=0.,points_world=np.empty((0,3))))
    assert not np.any(p.act(observation(),np.array([2.,0.])))
    assert p.records[-1]['route_reason']=='no_known_free_route'

def test_packet_consumed_once_prevents_stale_feedback():
    p=m.InformationPolicy(P,'visible');p.reset(None);p.set_current_packet(dict(time_s=0.,points_world=np.empty((0,3))))
    p.act(observation(),np.array([2.,0.]))
    import pytest
    with pytest.raises(RuntimeError):p.act(observation(),np.array([2.,0.]))

def test_visible_cannot_receive_full_map():
    import pytest
    p=m.InformationPolicy(P,'visible')
    with pytest.raises(PermissionError):p.set_current_packet({'known_grid':{},'time_s':0.,'points_world':np.empty((0,3))})

def test_future_files_cannot_change_current_action(tmp_path):
    def action():
        p=m.InformationPolicy(P,'visible');p.belief=m.BeliefGrid([-5.,-5.],shape=(100,100));p.belief.labels[:]=0
        p.set_current_packet(dict(time_s=0.,points_world=np.empty((0,3))))
        return p.act(observation(),np.array([2.,0.]))
    original=action();future=tmp_path/'future.npz'
    np.savez(future,position=np.ones((99,2))*1e6,request=np.ones((99,10))*1e6)
    assert np.array_equal(original,action())
    future.unlink()
    assert np.array_equal(original,action())
