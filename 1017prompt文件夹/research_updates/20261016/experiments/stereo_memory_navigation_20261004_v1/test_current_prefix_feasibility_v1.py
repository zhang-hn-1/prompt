from pathlib import Path
import numpy as np
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from current_prefix_feasibility_v1 import current_two_forward_prefix
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1'
def fixture(i):
 import json
 m=np.load(D/'maps'/f'frame_{i:03d}.npz');g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'].copy();s=json.loads((D/'states.json').read_text())[i];from scipy.spatial.transform import Rotation
 R=Rotation.from_quat(np.asarray(s['quaternion'])[[1,2,3,0]]).as_matrix();return g,s['position'][:2],float(np.arctan2(R[1,0],R[0,0]))
def test_actual_blockage_no_map_mutation():
 g,xy,yaw=fixture(506);old=g.labels.copy();r=current_two_forward_prefix(g,xy,yaw);assert r['tested_pairs']==9 and r['feasible_pairs']==0 and not r['pass'];assert np.array_equal(old,g.labels)
def test_current_earlier_state_not_future():
 g,xy,yaw=fixture(471);r=current_two_forward_prefix(g,xy,yaw);assert r['feasible_pairs']==9 and r['pass'];assert not r['physical_or_safety_certificate']
def test_translated_current_geometry_covariance():
 g,xy,yaw=fixture(506);a=current_two_forward_prefix(g,xy,yaw);delta=np.array([2.,-3.]);g.origin=g.origin+delta;b=current_two_forward_prefix(g,np.array(xy)+delta,yaw);assert a['feasible_pairs']==b['feasible_pairs']==0
def test_composed_boundary_replans_without_act_or_latch_change():
 from types import SimpleNamespace
 from current_prefix_reference_consumer_v1 import with_current_prefix_after_measured_reversal
 from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
 from benchmark.drive_model import DriveModel
 import json
 class Base:pass
 cls=with_first_primitive_direction(with_current_prefix_after_measured_reversal(Base));p=cls();g,xy,yaw=fixture(501);p.grid=g;p.plan=object();p.plan_meta={};p.tube_viability_armed=True;p.current_first_direction='forward';p.local_reference_version=6;p.reference_installed_time=38.6;p.first_direction_query=None
 r=json.loads((D/'policy_records.json').read_text());p.tracker=SimpleNamespace(_path_index=3,_active_drive_direction='forward',_pending_drive_direction=None,_stopped_cycles=0,_off_reference_cycles=0,_failure_subreason=None,_braking_state='tracking',mapping_records=[],_context=object(),_reference_xy=np.asarray(r[386]['installed_reference_xy_after_act']),drive_model=DriveModel.from_json('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json'))
 old=g.labels.copy();created,decision=p.plan_boundary(np.array(xy),yaw,np.array([3.300000005960465,5.960465e-9]),50.1);assert created and p.plan is None;assert decision['tube_viability_screen']['pass'] and not decision['current_prefix_screen']['pass'];assert p.first_direction_query['direction']=='forward' and p.first_direction_query['status']=='unreachable';assert p.tracker._active_drive_direction=='forward' and p.tracker._path_index==3 and p.local_reference_version==7;assert np.array_equal(g.labels,old)
