import copy
import numpy as np
from execution_snapshot import ExecutionClone,export_execution
from guarded_response import GuardedResponseConsumer,physical_state,make_entry
from training.action_protocol import load_action_protocol
from scripts.action_smoothing import ActionSmoother
from roll_pid import BankPID,FeedbackSmoother
from types import SimpleNamespace

def wrapper():
    return FeedbackSmoother(ActionSmoother(.02,max_auxiliary_velocity_rad_s=.3),SimpleNamespace(),BankPID(.4,.1,.7),[])
def protocol():
    return load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
def state(yaw=0):
    q=np.array([np.cos(yaw/2),0,0,np.sin(yaw/2)])
    return physical_state([0,0,.45],q,[0,0,0],[0,0,0],[0,0,0],[0,0,0])
def entry():
    execution=export_execution(wrapper(),protocol())
    return make_entry('positive',state(),execution,np.array([[0,0],[.1,-.15],[.6,-.27],[.7,-.1]]),np.tile([.3,.5]*5,(120,1)),np.array([0,1.5,6,12.]))

def test_complete_clone_supervisor_feedback_and_readonly_snapshot():
    w=wrapper();p=protocol();before=export_execution(w,p);c=ExecutionClone.from_snapshot(before)
    applied,smooth=c.step([.3,.6],bank_rad=.1,aux_position_rad=.03,aux_velocity_rad_s=.8)
    assert c.snapshot()['blocked'] and applied[1]==np.float32(.03)
    assert smooth[1]!=applied[1]
    assert export_execution(w,p)==before
    fork=ExecutionClone.from_snapshot(c.snapshot())
    for bank in [.1,.11,.09]:
        a,s=c.step([0,0],bank_rad=bank,aux_position_rad=.025,aux_velocity_rad_s=.2)
        b,t=fork.step([0,0],bank_rad=bank,aux_position_rad=.025,aux_velocity_rad_s=.2)
        np.testing.assert_array_equal(a,b);np.testing.assert_array_equal(s,t)
    altered=copy.deepcopy(before);altered['pid']['rate']=.2
    assert ExecutionClone.from_snapshot(altered).snapshot()['pid']['rate']==.2

def test_measured_xy_rotates_with_current_yaw_and_full_tail_is_checked():
    e=entry();consumer=GuardedResponseConsumer([e]);labels=np.full((101,101),-1,np.int8)
    result=consumer.select(state(np.pi/2),e['execution'],[.1,.7],labels,[-5,-5],.1)
    assert result['selected']=='positive' and result['status']=='unknown_risk'
    np.testing.assert_allclose(result['world_xy'],[[0,0],[.15,.1],[.27,.6],[.1,.7]],atol=1e-10)
    assert not result['full_3d_certified'] and len(result['requests'])==120
    # A hazard hit only by the whole matching stopping tail rejects the plan.
    e['xy']=np.array([[0,0],[.1,0],[.2,0],[1.5,0]])
    labels[65,50]=1
    r=consumer.select(state(),e['execution'],[2.,0],labels,[-5,-5],.1)
    assert r['selected'] is None and r['status']=='recorded_hazard'
    assert labels[50,50]==-1

def test_actual_state_pid_and_smoother_are_applicability_inputs():
    e=entry();c=GuardedResponseConsumer([e]);grid=np.full((31,31),-1,np.int8)
    for field,value in [('linear_local',[.12,0,0]),('joint_velocity',[.2,0,0]),('tilt_q',[np.cos(.1),np.sin(.1),0,0])]:
        s=state();s[field]=value
        r=c.select(s,e['execution'],[1,0],grid,[-1.5,-1.5],.1)
        assert r['selected'] is None and r['status']=='unsupported_initial_state'
    for key,val in [('blocked',True),('pid',dict(e['execution']['pid'],rate=.2)),('inner',dict(e['execution']['inner'],current_action=[.1,0]))]:
        x=copy.deepcopy(e['execution']);x[key]=val
        assert c.select(state(),x,[1,0],grid,[-1.5,-1.5],.1)['selected'] is None
    del x['inner']
    assert c.select(state(),x,[1,0],grid,[-1.5,-1.5],.1)['selected'] is None

def test_consumer_only_consumes_prespecified_requests_no_future_feedback():
    e=entry();c=GuardedResponseConsumer([e]);g=np.full((31,31),-1,np.int8)
    r=c.select(state(),e['execution'],[1,-.3],g,[-1.5,-1.5],.1)
    e['future_actual_feedback_poison']='must never be read'
    s=c.select(state(),e['execution'],[1,-.3],g,[-1.5,-1.5],.1)
    np.testing.assert_array_equal(r['requests'],s['requests']);np.testing.assert_array_equal(r['world_xy'],s['world_xy'])
    # The caller cannot mutate the stored response through returned arrays.
    s['world_xy'][:]=999
    t=c.select(state(),e['execution'],[1,-.3],g,[-1.5,-1.5],.1)
    np.testing.assert_array_equal(r['world_xy'],t['world_xy'])

def test_future_packet_tail_is_not_read_by_decision_replay():
    from guarded_decision_replay import replay_memory
    packets=list(range(181));read=[]
    def loader(i):
        if i>60:raise AssertionError('future packet read')
        read.append(i)
        return dict(root_xyz=np.array([0,0,.45]),time_s=i*.1,points_world=np.array([[.8,0,0.],[1.1,0,.3]]) if i==30 else np.empty((0,3)))
    a=replay_memory(packets,60,loader)
    assert read==list(range(61))
    b=replay_memory(packets[:61],60,loader)
    poison=packets[:61]+['future logs removed and replaced']
    c=replay_memory(poison,60,loader)
    np.testing.assert_array_equal(a.labels(6.),b.labels(6.))
    np.testing.assert_array_equal(a.labels(6.),c.labels(6.))
    assert a.status([1.1,0],6.)=='occupied'
