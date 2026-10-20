import numpy as np
from short_branch import development_schedule,CompletionGate,whole_short_entry
from test_guarded_response import entry

def test_short_prefix_has_its_own_complete_zero_tail():
    a=development_schedule()
    assert a.shape==(135,10) and not a[:60].any() and not a[75:].any()
    np.testing.assert_allclose(a[60:75].reshape(-1,2),np.tile([.3,np.pi/6],(75,1)),atol=1e-7)
    e=whole_short_entry(entry(),np.array([[0,0],[.1,-.15],[.2,-.02]]),np.array([0,1.5,7.5]),a[60:])
    assert len(e['requests'])==75 and e['scope'].startswith('1.5s')
    # Cannot invent a stop from the historical6s-turn response by early truncation.
    try:whole_short_entry(entry(),np.zeros((2,2)),np.array([0,1.5]),a[60:])
    except ValueError:pass
    else:raise AssertionError('truncated stop horizon admitted')

def test_plan_completion_and_actual_stop_are_separate_events():
    g=CompletionGate(75)
    for _ in range(74):assert not g.observe(.01)['decision_boundary']
    assert g.observe(.01)['decision_boundary']
    assert not g.observe(.06)['decision_boundary'] and g.low_count==0
    for _ in range(4):assert not g.observe(.01)['decision_boundary']
    assert g.observe(.01)['decision_boundary']

def test_zero_request_does_not_qualify_and_no_feedback_reset_is_implied():
    g=CompletionGate(75)
    for _ in range(75):r=g.observe(.12)
    assert r['plan_exhausted'] and not r['actual_stop_qualified'] and not r['decision_boundary']
    assert g.executed_blocks==75 and g.low_count==0

def test_live_short_consumer_returns_short_matched_requests():
    from short_branch import ShortConsumer
    e=whole_short_entry(entry(),np.array([[0,0],[.1,-.15],[.2,-.02]]),np.array([0,1.5,7.5]),development_schedule()[60:])
    c=ShortConsumer([e]);grid=np.full((41,41),-1,np.int8)
    r=c.select(e['state'],e['execution'],[.4,-.15],grid,[-2,-2],.1)
    assert r['selected']=='short_positive' and len(r['requests'])==75
    assert r['scope'].startswith('1.5s') and not r['full_3d_certified']
