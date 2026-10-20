"""Catch world-Z being substituted for projected-heading rate and state leakage."""
import importlib
import math
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

LEGACY = Path(__file__).resolve().parents[1] / 'stereo_memory_navigation_20261004_v1'
PROJECT = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(LEGACY))

def api():
    return importlib.import_module('heading_rate')

def quaternion(pitch=0., yaw=0., roll=0.):
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()[[3,0,1,2]]

def test_tilted_body_world_x_rotation_changes_horizontal_heading():
    got=api().projected_heading_rate(quaternion(pitch=math.pi/6), [.04,0.,0.])
    assert got['supported']
    assert got['yaw_rate_rad_s']==pytest.approx(.04/math.sqrt(3), abs=1e-12)
    assert got['world_z_rate_rad_s']==0.

def test_pure_world_z_rotation_and_quaternion_sign_invariance():
    q=quaternion(pitch=.4,yaw=.7,roll=.6)
    a=api().projected_heading_rate(q,[0.,0.,.2])
    b=api().projected_heading_rate(-q,[0.,0.,.2])
    assert a['yaw_rate_rad_s']==pytest.approx(.2,abs=1e-12)
    assert b==a

def test_horizontal_axis_reproduces_world_z_despite_roll():
    got=api().projected_heading_rate(quaternion(roll=.8),[.1,.2,.3])
    assert got['yaw_rate_rad_s']==pytest.approx(.3,abs=1e-12)

def test_vertical_axis_is_unknown_not_a_fabricated_zero():
    got=api().projected_heading_rate(quaternion(pitch=math.pi/2),[.1,.2,.3])
    assert not got['supported'] and got['yaw_rate_rad_s'] is None

@pytest.mark.parametrize('q,w', [([1,0,0,0],[0,np.nan,0]),([0,0,0,0],[0,0,0]),([2,0,0,0],[0,0,0]),([1,0,0],[0,0,0])])
def test_invalid_current_kinematics_rejected(q,w):
    with pytest.raises(ValueError):
        api().projected_heading_rate(q,w)

def tracker(enabled, direction='forward'):
    from scripts.run_pointgoal_benchmark import load_action_protocol
    from outgoing_lookahead import make_endpoint_oracle
    from benchmark.kinodynamic_planning import KinodynamicWaypoint,KinodynamicPlanResult
    from depth_local_curve import install_local_reference
    protocol=load_action_protocol(PROJECT/'configs/data_contract.yaml')
    original=make_endpoint_oracle(protocol)
    original.__class__=api().with_heading_rate_tracker(type(original),enabled=enabled)
    sign=1 if direction=='forward' else -1
    points=tuple(KinodynamicWaypoint(sign*x,0.,0.,direction,sign*.5,0.) for x in [0.,.3,.6,.9,1.2,1.5])
    install_local_reference(original,KinodynamicPlanResult('success',points,1,1.5,0))
    return original

def observation(*,pitch=math.pi/6, angular=(.04,0.,0.), velocity=(.1,0.,0.)):
    from benchmark.policies import PolicyObservation
    prop=np.zeros((5,17));prop[:,:3]=velocity;prop[:,3:6]=angular
    q=quaternion(pitch=pitch)
    return PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),prop,np.zeros((5,10)),np.zeros(2),0.,quaternion_world_wxyz_history=np.tile(q,(5,1)))

@pytest.mark.parametrize('direction',['forward','reverse'])
def test_real_tracker_only_correction_changes_and_consumes_one_act(direction):
    sign=1 if direction=='forward' else -1
    a=tracker(False,direction);b=tracker(True,direction)
    o=observation(velocity=(sign*.1,0.,0.));q_before=o.quaternion_world_wxyz_history.copy();p_before=o.proprio_history.copy()
    old=a.act(o,[3.,0.]);new=b.act(o,[3.,0.])
    np.testing.assert_array_equal(old[::2],new[::2])
    assert abs(float(new[1]-old[1])-sign*b.yaw_rate_feedback_gain_s*.04/math.sqrt(3))<1e-7
    assert len(a.tracking_records)==len(b.tracking_records)==1
    assert a.path_index==b.path_index and a._active_drive_direction==b._active_drive_direction
    assert a._pending_drive_direction==b._pending_drive_direction
    np.testing.assert_array_equal(o.proprio_history,p_before)
    np.testing.assert_array_equal(o.quaternion_world_wxyz_history,q_before)
    assert b._heading_rate_current is None
    diag=b.tracking_diagnostics
    assert diag['heading_rate_consumer_applied'] and diag['measured_world_yaw_rate_rad_s']==0.
    assert diag['measured_projected_heading_rate_rad_s']==pytest.approx(.04/math.sqrt(3))

def test_braking_and_goal_preserve_actual_parent_requests():
    for goal,pending in [([0.,0.],False),([3.,0.],True)]:
        a=tracker(False);b=tracker(True)
        if pending:
            a._active_drive_direction=b._active_drive_direction='reverse'
        old=a.act(observation(velocity=(.2,0.,0.)),goal)
        new=b.act(observation(velocity=(.2,0.,0.)),goal)
        np.testing.assert_array_equal(old,new)
        assert not b.tracking_diagnostics['heading_rate_consumer_applied']
        assert a._stopped_cycles==b._stopped_cycles and a._pending_drive_direction==b._pending_drive_direction

def test_upright_supported_and_singular_fallback_are_exact_parent_actions():
    for pitch in [0.,math.pi/2]:
        a=tracker(False);b=tracker(True)
        np.testing.assert_array_equal(a.act(observation(pitch=pitch),[3.,0.]),b.act(observation(pitch=pitch),[3.,0.]))

def test_current_observation_cleared_if_inherited_act_raises():
    b=tracker(True)
    with pytest.raises(ValueError):
        b.act(observation(),[float('nan'),0.])
    assert b._heading_rate_current is None

def test_correction_keeps_existing_five_degree_clip():
    b=tracker(True)
    b.act(observation(angular=(10.,0.,0.)),[3.,0.])
    assert b.tracking_diagnostics['heading_rate_feedback_after_rad']==pytest.approx(math.radians(5))

def test_quaternion_and_navigation_heading_mismatch_rejected_at_tracking():
    b=tracker(True);o=observation()
    from dataclasses import replace
    with pytest.raises(ValueError,match='heading mismatch'):
        b.act(replace(o,yaw_world_rad=.2),[3.,0.])
    assert b._heading_rate_current is None

@pytest.mark.parametrize('frame',[140,200])
def test_real_policy_factory_current_archived_boundary_matches_parent(frame):
    import json
    from benchmark.policies import PolicyObservation,EpisodeContext
    from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
    from scripts.run_pointgoal_benchmark import load_action_protocol
    from depth_local_curve import install_local_reference
    from prior_rgb_navigation import PriorRGBPolicy
    from execution_domain_four_point_consumer_v2 import with_supported_four_points
    from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
    from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
    from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
    from command_domain_cap import with_command_domain_cap
    from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
    run=LEGACY/'dense_current_live_e1_b0_run_v1'
    rows=json.loads((run/'policy_records.json').read_text());row=rows[frame];previous=rows[frame-1]['tracker_state_after_act']
    state=json.loads((run/'states.json').read_text())[frame]
    saved=json.loads((run/f'local_plans/frame_{frame:03d}.json').read_text())
    hist=dict(np.load(run/f'history/frame_{frame:03d}.npz'))
    R=Rotation.from_quat(np.array(state['quaternion'])[[1,2,3,0]]).as_matrix()
    yaw=float(np.arctan2(R[1,0],R[0,0]));xy=np.array(state['position'][:2]);t=row['time_s']
    o=PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),hist['proprio_history'],hist['action_history'],xy,yaw,quaternion_world_wxyz_history=hist['quaternion_world_wxyz_history'])
    goal=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])@(np.array([3.300000005960465,0.])-xy)
    protocol=load_action_protocol(PROJECT/'configs/data_contract.yaml')
    base=with_command_domain_cap(with_cusp_coast_acquisition(with_first_primitive_direction(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(PriorRGBPolicy)))))
    variants=[]
    for enabled in [False,True]:
        p=api().with_heading_rate_policy(base,enabled=enabled)(protocol,memory_mode='current')
        p.reset(EpisodeContext('diagnostic',False,None,protocol.protocol_id))
        plan=KinodynamicPlanResult(saved['status'],tuple(KinodynamicWaypoint(**w) for w in saved['waypoints']),saved['expanded_nodes'],saved['path_length_m'],saved['direction_changes'])
        install_local_reference(p.tracker,plan);p.plan=plan;p.plan_meta=saved['plan_meta'];p.last_plan=t;p.last_frame=frame-1;p.time=t-.1
        p.reference_decision=row['reference_decision'];p.reference_installed_time=row['reference_installed_time_s']
        for field,key in [('_path_index','path_index'),('_active_drive_direction','active_direction'),('_pending_drive_direction','pending_direction'),('_stopped_cycles','stopped_cycles'),('_off_reference_cycles','off_reference_cycles'),('_braking_state','braking_state')]:
            setattr(p.tracker,field,previous[key])
        p.grid=RepeatedExposureMemoryGrid(xy-20,10,static_scene_assumption=True)
        p.set_source_packet(dict(frame=frame,time_s=t,records=[]))
        p.set_current_packet(dict(frame=frame,time_s=t,points_world=np.empty((0,3)),root_xyz=np.array(state['position'])))
        a=p.act(o,goal);variants.append((p,a))
    original,new=variants
    np.testing.assert_array_equal(original[1],np.array(row['request'],np.float32))
    np.testing.assert_array_equal(original[1][::2],new[1][::2])
    assert len(original[0].tracker.tracking_records)==len(new[0].tracker.tracking_records)==1
    assert original[0].tracker.path_index==new[0].tracker.path_index
    assert original[0].tracker._pending_drive_direction==new[0].tracker._pending_drive_direction
    assert new[0].records[-1]['heading_rate_fresh_tracker_record']
    np.testing.assert_array_equal(original[0].tracker._reference_xy,new[0].tracker._reference_xy)
