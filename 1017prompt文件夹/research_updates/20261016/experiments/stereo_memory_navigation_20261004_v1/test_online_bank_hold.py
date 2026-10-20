import numpy as np
from online_bank_hold import OnlineBankHold

def block(main,aux):return np.tile(np.array([main,aux],np.float32),5)

def test_startup_and_request_transition_hold_last_emitted_motion_bank():
    c=OnlineBankHold()
    a,info=c.apply(block(0,0));assert not a.any() and not info['active']
    c.apply(block(.3,.2));c.apply(block(.3,-.4))
    source=block(0,0);before=source.copy();a,info=c.apply(source)
    np.testing.assert_array_equal(source,before)
    np.testing.assert_array_equal(a,block(0,-.4))
    assert info['entered'] and info['active'] and info['source']=='previous_emitted_motion_request'
    a,info=c.apply(block(0,.1));np.testing.assert_array_equal(a,block(0,-.4));assert not info['entered']

def test_resume_does_not_wait_for_qualification_or_force_direction():
    c=OnlineBankHold();c.apply(block(.3,.4));c.apply(block(0,0))
    reverse=block(-.3,-.2);a,info=c.apply(reverse)
    np.testing.assert_array_equal(a,reverse);assert not info['active']
    a,info=c.apply(block(0,0));np.testing.assert_array_equal(a,block(0,-.2));assert info['entered']

def test_past_only_latch_snapshot_is_readonly_and_future_tail_irrelevant():
    commands=[block(0,0),block(.3,.4),block(0,0)]
    def replay(seq):
        c=OnlineBankHold();out=[]
        for a in seq[:3]:out.append(c.apply(a)[0])
        before=c.snapshot();copy=c.snapshot();copy['previous_motion_aux'][0]=999
        assert c.snapshot()==before
        return out,before
    a,sa=replay(commands);b,sb=replay(commands+[object()])
    for x,y in zip(a,b):np.testing.assert_array_equal(x,y)
    assert sa==sb

def test_live_policy_consumes_parent_once_and_keeps_low_speed_counter():
    from types import SimpleNamespace
    from online_bank_hold_policy import InformationPolicy
    from training.action_protocol import load_action_protocol
    protocol=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
    policy=InformationPolicy(protocol,'visible','points',memory_mode='history')
    obs=SimpleNamespace(position_world_xy=np.zeros(2),yaw_world_rad=.2)  # Nonzero tracking heading error, independent of A* first axis.
    def update(time,points):
        policy.set_current_packet(dict(time_s=time,points_world=np.asarray(points).reshape(-1,3),root_xyz=np.array([0.,0.,.4]),speed_xy=.12))
        return policy.act(obs,np.array([3.,-.8]))
    a=update(6.,[]);assert a[0]!=0 and a[1]!=0
    b=update(6.5,[[.7,0,.4]])
    assert len(policy.records)==2 and policy._packet is None and policy.stopping.low_count==0
    np.testing.assert_array_equal(b[0::2],np.zeros(5,np.float32));np.testing.assert_array_equal(b[1::2],a[1::2])
    assert policy.records[-1]['stopping_consumer']['stop_veto']
    assert policy.records[-1]['bank_stop_hold']['entered']
