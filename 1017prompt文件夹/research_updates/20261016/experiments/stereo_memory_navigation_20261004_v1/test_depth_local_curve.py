from pathlib import Path
import sys
import numpy as np
import pytest
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
import depth_local_curve as m
from benchmark.policies import PolicyObservation,EpisodeContext
from training.action_protocol import load_action_protocol
from provenance_grid import ProvenanceGrid
from oracle_rgb_shadow import make_oracle,reference_from_arrays

def packet(frame=0,t=0.,points=()):
 return dict(frame=frame,time_s=t,points_world=np.asarray(points,float).reshape(-1,3),root_xyz=np.array([0.,0.,.4]))
def obs(x=0.):
 rgb=np.zeros((480,640,3),np.uint8)
 return PolicyObservation(rgb,rgb,np.zeros((2,17),np.float32),np.zeros((2,10),np.float32),np.array([x,0.]),0.)
def protocol():return load_action_protocol(P/'configs/data_contract.yaml')
def test_packet_no_privileged_alias_and_bad_clock():
 # Break: forwarding known grid/array aliases or accepting NaN introduces truth or noncausal evidence.
 p=packet(points=[[1.,0.,0.]])
 with pytest.raises(ValueError):m.sanitize_packet(dict(p,known_grid=np.zeros((4,4))))
 with pytest.raises(ValueError):m.sanitize_packet(packet(t=np.nan))
 s=m.sanitize_packet(p);s['points_world'][:]=10.;assert p['points_world'][0,0]==1.

def test_empty_evidence_remains_unknown_despite_risk_plan():
 # Break: body/free seed or treating planning copy as memory changes never-observed support.
 g=ProvenanceGrid([-20,-20]);inner=make_oracle(protocol())
 plan,meta=m.local_curve(g,np.array([0.,0.]),0.,np.array([4.,0.]),inner.drive_model)
 assert plan.status=='success' and plan.path_length_m<=2.7
 assert np.all(g.labels==-1) and meta['unknown_fraction']==1. and not meta['certified_safe']
 # Frozen measured V1 has nonzero curvature even for aux=0; do not substitute an ideal straight robot.
 assert np.linalg.norm(np.array([plan.waypoints[-1].x_m,plan.waypoints[-1].y_m])-[2.4,0.])<=.2

def test_observed_hazard_at_start_blocks_motion():
 # Break: ignoring obstacle evidence or overriding blocked start admits nominal collision.
 g=ProvenanceGrid([-20,-20]);g.update_points([[0.,0.,.8]],[10,10,.4],0.)
 plan,meta=m.local_curve(g,np.array([0.,0.]),0.,np.array([4.,0.]),make_oracle(protocol()).drive_model)
 assert plan is None and meta['status']=='observed_obstacle_blocks_route'

def test_reference_update_preserves_actual_direction_confirmation():
 # Break: reset inner policy per replanning loses two already measured low-speed confirmations.
 inner=make_oracle(protocol());ctx=EpisodeContext('test',True,np.array([[0.,0.],[2.,0.]]),protocol().protocol_id,reference_from_arrays([[0,0,0],[2,0,0]],[1,1],[[.5,0],[0,0]],2.,1).waypoints);inner.reset(ctx)
 assert inner._handle_direction_change('reverse',.04) is not None
 assert inner._handle_direction_change('reverse',.04) is not None
 plan=reference_from_arrays([[0,0,0],[1,0,0]],[1,1],[[.5,0],[0,0]],1.,1)
 m.install_local_reference(inner,plan)
 assert inner._pending_drive_direction=='reverse' and inner._stopped_cycles==2
 assert inner._handle_direction_change('reverse',.04) is not None
 assert inner._handle_direction_change('reverse',.04) is not None
 assert inner._handle_direction_change('reverse',.04) is None and inner._active_drive_direction=='reverse'

def test_outer_denies_global_context_and_bad_packet_is_atomic():
 # Break: receiving runtime full reference or corrupting timestamp state on rejected packet.
 p=m.DepthLocalCurvePolicy(protocol())
 with pytest.raises(PermissionError):p.reset(EpisodeContext('bad',True,np.array([[0.,0.],[2.,0.]]),protocol().protocol_id))
 p.reset(EpisodeContext('ok',False,None,protocol().protocol_id));p.set_current_packet(packet())
 np.testing.assert_equal(p.act(obs(),[4,0]),np.zeros(10))
 with pytest.raises(ValueError):p.set_current_packet(packet(1,-1.))
 p.set_current_packet(packet(1,.1));p.act(obs(),[4,0]);assert p.records[-1]['time_s']==.1

def test_real_curve_tracking_enters_frozen_mapping_without_body_seed():
 # Break: leaving the old heading tracker in place or never consuming the locally planned reference.
 p=m.DepthLocalCurvePolicy(protocol());p.reset(EpisodeContext('ok',False,None,protocol().protocol_id))
 p.set_current_packet(packet(0,6.));action=p.act(obs(),[4,0])
 assert action[0]>0 and p.tracker.mapping_records[-1]['active']
 assert p.records[-1]['local_plan_created'] and p.records[-1]['unknown_fraction']==1.
 assert np.all(p.grid.labels==-1) and p.tracker._context.oracle_path_xy[-1,0]<=2.6

def test_future_packet_payload_is_never_read():
 # Break: reading future points before causal clock filter leaks or triggers trap.
 class Trap(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return super().__getitem__(k)
 a=list(m.causal_packets([packet(),Trap(time_s=.2)],.1))
 assert len(a)==1 and a[0]['frame']==0

def test_failed_plan_does_not_trigger_per_frame_retry():
 # Break: retrying a known blocked start every0.1s instead of the frozen0.5s cadence increases search budget.
 p=m.DepthLocalCurvePolicy(protocol());p.reset(EpisodeContext('ok',False,None,protocol().protocol_id))
 for i,t in enumerate([6.,6.1]):
  p.set_current_packet(packet(i,t,[[.4,0.,.8]]));p.act(obs(),[4,0])
 assert p.records[0]['local_plan_created'] and not p.records[1]['local_plan_created']
