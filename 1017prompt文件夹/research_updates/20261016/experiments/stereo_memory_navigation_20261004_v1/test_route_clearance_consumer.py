import numpy as np
from provenance_grid import ProvenanceGrid
from route_clearance_consumer import RouteClearanceGrid, STOP_RADIUS_M

def grids(hazard=None):
    old=ProvenanceGrid([-5.,-5.],shape=(100,100));new=RouteClearanceGrid([-5.,-5.],shape=(100,100))
    if hazard is not None:
        cell=old.cells(np.array(hazard)[None])[0]
        old.labels[tuple(cell)]=1;new.labels[tuple(cell)]=1
    return old,new

def test_no_hazard_preserves_original_route_and_unknown_labels():
    old,new=grids();before=new.labels.copy()
    a,ra=old.plan([0,0],[3,0]);b,rb=new.plan([0,0],[3,0])
    np.testing.assert_array_equal(a,b);assert ra==rb
    assert not new.clearance_active
    np.testing.assert_array_equal(before,new.labels)

def test_live_route_event_activates_matching_radius_and_persists_without_label_writes():
    old,new=grids([1.1,0]);before=new.labels.copy()
    a,_=old.plan([0,0],[3,0]);b,_=new.plan([0,0],[3,0])
    assert new.clearance_active and new.clearance_record['entered']
    assert new.clearance_record['current_hazard_clearance_m'] > STOP_RADIUS_M
    assert new.clearance_record['old_local_path_clearance_m'] < STOP_RADIUS_M
    assert not np.array_equal(a,b)
    assert new.clearance_record['required_point_clearance_m']==STOP_RADIUS_M
    np.testing.assert_array_equal(before,new.labels)
    new.labels[:]=-1;new.plan([0,0],[3,0])
    assert new.clearance_active and not new.clearance_record['entered']

def test_current_stopping_disc_is_not_relaxed_to_enable_activation():
    old,new=grids([.7,0]);a,ra=old.plan([0,0],[3,0]);b,rb=new.plan([0,0],[3,0])
    np.testing.assert_array_equal(a,b);assert ra==rb
    assert not new.clearance_active

def test_actual_point_clearance_does_not_override_blocked_rounded_cell():
    _,new=grids([.7,0]);new.clearance_active=True
    p,reason=new.plan([-.049,0],[3,0])
    assert p is None and reason=='observed_obstacle_blocks_route'
    assert new.clearance_record['current_hazard_clearance_m'] > STOP_RADIUS_M
    assert new.clearance_record['start_grid_clearance_m'] < STOP_RADIUS_M

def test_policy_consumes_once_and_preserves_parent_before_event():
    from types import SimpleNamespace
    from route_clearance_policy import InformationPolicy
    from online_bank_hold_policy import InformationPolicy as Baseline
    from training.action_protocol import load_action_protocol
    protocol=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
    policies=[Baseline(protocol,'visible','points'),InformationPolicy(protocol,'visible','points')]
    obs=SimpleNamespace(position_world_xy=np.zeros(2),yaw_world_rad=.2)
    for time,points in [(6.,[]),(6.5,[[1.1,0,.4]])]:
        actions=[]
        for p in policies:
            p.set_current_packet(dict(time_s=time,points_world=np.asarray(points).reshape(-1,3),root_xyz=np.array([0,0,.4]),speed_xy=.12))
            actions.append(p.act(obs,np.array([3.,0.])))
            assert p._packet is None and len(p.records)==(1 if time==6. else 2)
        if time==6.:
            np.testing.assert_array_equal(actions[0],actions[1])
            assert policies[0].records[-1]=={k:v for k,v in policies[1].records[-1].items() if k!='route_clearance'}
        else:
            assert policies[1].records[-1]['route_clearance']['entered_this_call']
            assert policies[0].stopping.low_count==policies[1].stopping.low_count==0

def test_between_replans_geometry_is_explicitly_plan_time_not_current_feedback():
    from types import SimpleNamespace
    from route_clearance_policy import InformationPolicy
    from training.action_protocol import load_action_protocol
    protocol=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
    p=InformationPolicy(protocol,'visible','points')
    for time in (6.,6.1):
        p.set_current_packet(dict(time_s=time,points_world=np.empty((0,3)),root_xyz=np.array([0,0,.4]),speed_xy=.12))
        p.act(SimpleNamespace(position_world_xy=np.zeros(2),yaw_world_rad=.2),np.array([3,0]))
    info=p.records[-1]['route_clearance']
    assert info.get('geometry_current_for_this_call') is False
    assert info.get('geometry_measured_at_plan_time_s')==6.
    assert info.get('entered_this_plan') is False and not info['entered_this_call']
    assert 'entered' not in info
