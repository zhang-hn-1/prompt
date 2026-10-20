"""CPU real-consumer fixtures, separate from the one prospective physical episode."""
import copy,dataclasses
import numpy as np
import pytest
import root_direction_consumer as m
from depth_local_curve import local_curve,install_local_reference,P
from provenance_grid import ProvenanceGrid
from oracle_rgb_shadow import reference_from_arrays,make_oracle
from training.action_protocol import load_action_protocol
from benchmark.policies import EpisodeContext
from benchmark.kinodynamic_planning import KinodynamicWaypoint,KinodynamicPlanResult
from direction_root_planning import KinodynamicWaypoint as NewWaypoint,KinodynamicPlanResult as NewResult

def protocol():return load_action_protocol(P/'configs/data_contract.yaml')
def installed():
 p=m.RootDirectionPolicy(protocol());p.reset(EpisodeContext('fixture',False,None,p.protocol.protocol_id))
 p.grid=ProvenanceGrid([-20.,-20.],memory_mode='history')
 p.plan=reference_from_arrays([[0,0,0],[1,0,0],[2,0,0],[3,0,0]],[1]*4,[[.5,0]]*3+[[0,0]],3.,1)
 install_local_reference(p.tracker,p.plan);p.plan_meta=dict(status='success',unknown_fraction=1.)
 p.last_plan=6.;p.reference_installed_time=6.;p.local_reference_version=1
 return p

def test_none_direction_uses_same_old_toy_curve_and_original_types():
 # Break: copied search changes initial B0 geometry/units or leaks copied runtime types.
 grid=ProvenanceGrid([-20.,-20.],memory_mode='history');drive=make_oracle(protocol()).drive_model
 old,_=local_curve(grid,[0.,0.],0.,[4.,0.],drive)
 plan,meta=m.local_curve_for_direction(grid,[0.,0.],0.,[4.,0.],drive,initial_direction=None)
 assert isinstance(plan,KinodynamicPlanResult) and all(isinstance(w,KinodynamicWaypoint) for w in plan.waypoints)
 assert [dataclasses.asdict(w) for w in plan.waypoints]==[dataclasses.asdict(w) for w in old.waypoints]
 assert plan.expanded_nodes==old.expanded_nodes and plan.path_length_m==old.path_length_m
 assert meta['initial_direction'] is None and np.all(grid.labels==-1)

def test_retention_does_not_search_or_reset_any_tracker_state(monkeypatch):
 # Break: passing current direction calls search/reset even when old reference remains valid.
 p=installed();p.tracker._path_index=1;p.tracker._pending_drive_direction='reverse';p.tracker._stopped_cycles=2
 before=copy.deepcopy(p.tracker.__dict__)
 def forbidden(*a,**k):raise AssertionError('retention searched')
 monkeypatch.setattr(m,'local_curve_for_direction',forbidden)
 created,d=p.plan_boundary(np.array([1.2,.1]),0.,[4.,0.],6.5)
 assert not created and d['direction_cost_input'] is None and d['state_before']==d['state_after']
 assert p.tracker._reference==before['_reference'] and p.tracker._path_index==1 and p.tracker._stopped_cycles==2
 assert p.tracker._pending_drive_direction=='reverse' and len(p.tracker.mapping_records)==len(before['mapping_records'])
 assert p.local_reference_version==1 and p.reference_installed_time==6.

def test_rebuild_uses_accepted_before_install_and_preserves_five_low_rebound(monkeypatch):
 # Break: newly desired/pending F instead of accepted R enters root cost; installation clears lowcount.
 p=installed();p.tracker._active_drive_direction='reverse';p.tracker._desired_drive_direction='forward';p.tracker._pending_drive_direction='forward';p.tracker._stopped_cycles=2
 plan=reference_from_arrays([[2.6,0,0],[3.6,0,0]],[1,1],[[.5,0],[0,0]],1.,1)
 def search(grid,xy,yaw,goal,drive,*,initial_direction):
  assert initial_direction=='reverse';return plan,dict(status='success',unknown_fraction=1.,initial_direction=initial_direction)
 monkeypatch.setattr(m,'local_curve_for_direction',search)
 created,d=p.plan_boundary(np.array([2.6,0.]),0.,[4.,0.],6.5)
 assert created and d['direction_cost_input']=='reverse' and p.tracker._active_drive_direction=='reverse'
 assert p.tracker._stopped_cycles==2 and p.tracker._pending_drive_direction=='forward' and p.tracker._reference[0].drive_direction=='forward'
 assert p.tracker._handle_direction_change('forward',.06) is not None and p.tracker._stopped_cycles==0
 for _ in range(4):assert p.tracker._handle_direction_change('forward',.04) is not None
 assert p.tracker._handle_direction_change('forward',.04) is None and p.tracker._active_drive_direction=='forward'

def test_uninstalled_constructor_latch_is_not_accepted_motion(monkeypatch):
 # Break: blindly using default latch before any reference alters cold-start B0.
 p=m.RootDirectionPolicy(protocol());p.grid=ProvenanceGrid([-20.,-20.]);p.tracker._active_drive_direction='reverse'
 plan=reference_from_arrays([[0,0,0],[1,0,0]],[1,1],[[.5,0],[0,0]],1.,1)
 def search(grid,xy,yaw,goal,drive,*,initial_direction):
  assert initial_direction is None;return plan,dict(status='success',unknown_fraction=1.,initial_direction=None)
 monkeypatch.setattr(m,'local_curve_for_direction',search)
 created,d=p.plan_boundary(np.array([0.,0.]),0.,[4.,0.],6.)
 assert created and d['direction_cost_input'] is None and p.tracker._active_drive_direction=='forward'

def test_invalid_installed_latch_rejected_before_search(monkeypatch):
 # Break: converting invalid initialized direction to None silently discards execution history.
 p=installed();p.tracker._active_drive_direction='stopped'
 def forbidden(*a,**k):raise AssertionError('invalid latch reached search')
 monkeypatch.setattr(m,'local_curve_for_direction',forbidden)
 with pytest.raises(ValueError):p.plan_boundary(np.array([2.6,0.]),0.,[4.,0.],6.5)
 assert p.local_reference_version==1 and p.tracker._active_drive_direction=='stopped'

def test_terminal_only_success_is_no_usable_reference(monkeypatch):
 # Break: invoking old objective_summary on valid terminal-only result or installing it crashes runtime.
 def terminal(req,drive,*,initial_direction):
  return NewResult('success',(NewWaypoint(*req.start_xy,req.start_yaw_rad,'forward',0.,0.),),0,0.,0)
 monkeypatch.setattr(m,'plan_kinodynamic_path',terminal)
 plan,meta=m.local_curve_for_direction(ProvenanceGrid([-20.,-20.]),[0.,0.],0.,[0.,0.],make_oracle(protocol()).drive_model,initial_direction=None)
 assert plan is None and meta['status']=='success' and meta['local_path_length_m']==0.
