from pathlib import Path
import sys,copy
import numpy as np
import pytest
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from training.action_protocol import load_action_protocol
from benchmark.policies import EpisodeContext
from provenance_grid import ProvenanceGrid
from oracle_rgb_shadow import reference_from_arrays
from test_depth_local_curve import obs,packet
from depth_local_curve import install_local_reference
from guarded_reference import GuardedReferencePolicy

def installed(direction=1):
 p=GuardedReferencePolicy(load_action_protocol(P/'configs/data_contract.yaml'))
 p.reset(EpisodeContext('test',False,None,p.protocol.protocol_id))
 p.grid=ProvenanceGrid([-20.,-20.],memory_mode='history')
 p.plan=reference_from_arrays([[0,0,0],[1,0,0],[2,0,0],[3,0,0]],[direction]*4,[[.5*direction,0]]*3+[[0,0]],3.,1)
 install_local_reference(p.tracker,p.plan);p.plan_meta=dict(status='success',unknown_fraction=1.)
 p.last_plan=6.;p.reference_installed_time=6.;p.local_reference_version=1
 return p

def test_valid_old_reference_retains_index_direction_and_mapping_state():
 # Break: geometry reset on keep erases progress or braking history.
 p=installed();p.tracker._path_index=1;p.tracker._pending_drive_direction='reverse';p.tracker._stopped_cycles=2
 before=copy.deepcopy(p.tracker.__dict__);reference=p.tracker._reference
 created,decision=p.plan_boundary(np.array([1.2,.1]),0.,np.array([4.,0.]),6.5)
 assert not created and decision['decision']=='retain' and p.tracker._reference is reference
 assert p.tracker._path_index==1 and p.tracker._pending_drive_direction=='reverse' and p.tracker._stopped_cycles==2
 assert p.local_reference_version==1 and p.reference_installed_time==6.
 assert decision['state_before']==decision['state_after']
 assert p.tracker.__dict__.keys()==before.keys()

@pytest.mark.parametrize('xy,point,reason',[
 ([0.,0.],[1.5,0.,.8],'continuous_observed_hazard_clearance'),
 ([0.,.4],[0.,.3,.8],'continuous_observed_hazard_clearance'),
 ([0.,.6],None,'projection_distance_over_0p5'),
 ([2.6,0.],None,'remaining_arc_under_lookahead')])
def test_invalid_or_exhausted_old_reference_hands_to_real_planner(xy,point,reason):
 # Break: holding invalid/exhausted geometry indefinitely instead of original reconstruction.
 p=installed();old=p.tracker._reference
 if point is not None:p.grid.update_points([point],[10,10,.4],6.5)
 created,d=p.plan_boundary(np.asarray(xy),0.,np.array([4.,0.]),6.5)
 assert created and d['decision']=='rebuild' and reason in d['guard']['reject_reasons']
 assert p.local_reference_version==2
 assert p.reference_installed_time==(None if p.plan is None else 6.5)
 assert p.plan is None or p.tracker._reference is not old

def test_reverse_confirmation_survives_retention_and_rebound():
 # Break: retaining a route resets count or imposes a zero-speed prerequisite before reversal can begin.
 p=installed(-1);p.tracker._active_drive_direction='forward'
 for _ in range(2):assert p.tracker._handle_direction_change('reverse',.04) is not None
 p.plan_boundary(np.array([0.,0.]),0.,np.array([4.,0.]),6.5)
 assert p.tracker._stopped_cycles==2
 assert p.tracker._handle_direction_change('reverse',.06) is not None and p.tracker._stopped_cycles==0
 for _ in range(4):assert p.tracker._handle_direction_change('reverse',.04) is not None
 assert p.tracker._handle_direction_change('reverse',.04) is None and p.tracker._active_drive_direction=='reverse'

def test_real_act_retains_then_marks_guard_as_past_between_boundaries():
 # Break: act bypasses guard, or stale plan-time guard is labelled measured currently.
 p=GuardedReferencePolicy(load_action_protocol(P/'configs/data_contract.yaml'));p.reset(EpisodeContext('test',False,None,p.protocol.protocol_id))
 for i,t in enumerate([6.,6.5,6.6]):p.set_current_packet(packet(i,t));p.act(obs(),[4,0])
 assert p.records[0]['local_plan_created'] and not p.records[1]['local_plan_created']
 assert p.records[1]['reference_decision']['decision']=='retain'
 assert p.records[1]['guard_measured_this_call'] and not p.records[2]['guard_measured_this_call']
 assert p.records[2]['reference_decision']['measured_time_s']==6.5 and p.records[2]['reference_installed_time_s']==6.

def test_failed_rebuild_uses_old_cadence_without_reset_or_hidden_motion():
 # Break: failed rebuild retries per .1s or resumes old invalid plan.
 p=GuardedReferencePolicy(load_action_protocol(P/'configs/data_contract.yaml'));p.reset(EpisodeContext('test',False,None,p.protocol.protocol_id))
 for i,t in enumerate([6.,6.1]):
  p.set_current_packet(packet(i,t,[[.4,0.,.8]]));a=p.act(obs(),[4,0]);assert np.array_equal(a,np.zeros(10))
 assert p.records[0]['local_plan_created'] and not p.records[1]['local_plan_created']
 assert p.plan is None and p.records[1]['request']==[0.]*10
