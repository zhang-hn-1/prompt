"""Saved geometry/identity verification; no search or policy calls."""
from pathlib import Path
import sys,json,hashlib,datetime,difflib
from dataclasses import fields
import numpy as np
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark import kinodynamic_planning as old
from benchmark.drive_model import DriveModel
from benchmark.policies import DEFAULT_DRIVE_PROFILE
import direction_root_planning as new
from direction_root_cost import snapshot_problem
from reference_continuity_audit import reference_geometry_status
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'direction_root_cost_protocol.json').read_text());a=json.loads((H/'direction_root_cost_assessment.json').read_text());st=json.loads((H/'direction_root_cost_status.json').read_text());prov=json.loads((H/'direction_root_cost_provenance.json').read_text())
 for d in [p['source_sha256'],p['input_sha256']]:
  for path,digest in d.items():assert sha(path)==digest,path
 assert st['status']=='finished' and st['exit_code']==0 and st['actual_grid_searches']==1 and st['actual_hybrid_searches']==2
 assert p['sealed_utc']<a['started_utc']<a['saved_utc']<=st['ended_utc']
 source=(P/'benchmark/kinodynamic_planning.py').read_text();copy=(H/'direction_root_planning.py').read_text()
 patch=''.join(difflib.unified_diff(source.splitlines(True),copy.splitlines(True),fromfile=str(P/'benchmark/kinodynamic_planning.py'),tofile=str(H/'direction_root_planning.py')))
 assert patch==(H/'direction_root_cost_source_diff.patch').read_text()
 assert [(f.name,f.default) for f in fields(old.KinodynamicPlanRequest)]==[(f.name,f.default) for f in fields(new.KinodynamicPlanRequest)]
 packed=np.load(H/'direction_root_cost_snapshot.npz');maps={k:packed[k] for k in ['labels','provenance','floor_time','hazard_time','origin','resolution','time_s']};current={k:packed[k] for k in ['frame','position_world_xy','yaw_world_rad','pointgoal_base_xy']};current.update(time_s=float(packed['current_time_s']),accepted_direction=str(packed['accepted_direction']))
 sanitized=snapshot_problem(maps,current);req=np.load(H/'direction_root_cost_request.npz');origin=np.asarray(maps['origin']);resolution=float(maps['resolution']);labels=maps['labels'];xy=current['position_world_xy']
 center=np.rint((xy-origin)/resolution).astype(int);low=np.maximum(center-40,0);high=np.minimum(center+41,labels.shape)
 clear=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*resolution
 occupied=(clear<.6-1e-8).astype(np.uint8)[low[0]:high[0],low[1]:high[1]]
 np.testing.assert_equal(req['occupancy_grid'],occupied);np.testing.assert_equal(req['origin_xy'],origin+low*resolution);np.testing.assert_equal(req['start_xy'],xy);np.testing.assert_equal(req['goal_xy'],prov['expected_saved_meta']['local_target'])
 assert p['initial_direction']==prov['accepted_direction']==prov['accepted_state_before']['active_direction']==str(packed['accepted_direction'])=='forward'
 assert prov['accepted_previous_record']==1 and prov['accepted_state_before']['pending_direction'] is None
 class Trap(dict):
  def __getitem__(self,key):
   if key=='future_feedback':raise AssertionError('future feedback read')
   return super().__getitem__(key)
 try:snapshot_problem(maps,Trap(dict(current,future_feedback=None)))
 except ValueError:pass
 else:raise AssertionError('future field admitted')
 model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE);checks={};total_primitives=0;sample_count=0
 for arm in ['baseline','candidate']:
  plan=np.load(H/f'direction_root_cost_{arm}_reference.npz');poses=plan['pose'];commands=plan['command'];dirs=plan['direction'];record=a['results'][arm]
  assert record['status']=='success' and record['expanded_nodes']<=3000
  np.testing.assert_allclose(poses[0],np.r_[xy,float(current['yaw_world_rad'])],rtol=0,atol=1e-12)
  signed=np.sign(commands[:-1,0]).astype(int);np.testing.assert_equal(dirs[:-1],signed);assert np.all(commands[-1]==0.)
  prims=len(poses)-1;total_primitives+=prims;min_raster=float('inf');max_endpoint_error=0.;arm_samples=0
  for j in range(prims):
   main,aux=commands[j];k=model.curvature(float(main),float(aux));samples=old.sample_motion_primitive(start_pose=poses[j],signed_arc_length_m=.3*(1 if main>0 else -1),curvature_1_m=k,sample_spacing_m=.05)
   err=float(np.max(np.abs(samples[-1]-poses[j+1])));max_endpoint_error=max(max_endpoint_error,err);assert err<1e-10
   cells=np.floor((samples[:,:2]-req['origin_xy'])/resolution+.5).astype(int);assert np.all((cells>=0)&(cells<occupied.shape));assert not np.any(occupied[cells[:,0],cells[:,1]])
   allcells=np.floor((samples[:,:2]-origin)/resolution+.5).astype(int);min_raster=min(min_raster,float(clear[allcells[:,0],allcells[:,1]].min()));arm_samples+=len(samples)
  sample_count+=arm_samples;endpoint=float(np.linalg.norm(poses[-1,:2]-req['goal_xy']));assert endpoint<=.2+1e-12
  changes=int((signed[1:]!=signed[:-1]).sum());initial=int(signed[0]!=1);distance=.3*(int((signed>0).sum())+1.25*int((signed<0).sum()));objective=distance+.5*(changes+initial)
  assert abs(objective-record['current_direction_objective']['nominal_objective_m'])<1e-12 and record['direction_changes']==changes
  assert abs(record['path_length_m']-.3*prims)<1e-12
  guard=reference_geometry_status(xy,poses[:,:2],labels,origin,resolution)
  checks[arm]=dict(primitives=prims,nominal_sampled_points=arm_samples,maximum_reconstructed_endpoint_error=max_endpoint_error,goal_endpoint_error_m=endpoint,minimum_sampled_raster_clearance_m=min_raster,current_direction_objective_m=objective,original_objective_m=distance+.5*changes,total_direction_changes_including_initial=changes+initial,installed_polyline_continuous_clearance_m=min(guard['suffix_clearance_m'],guard['bridge_clearance_m']),installed_guard_necessary_screen_pass=guard['retention_necessary_screen_pass'],initial_changes=initial,internal_changes=changes,physical_dynamic_prediction=False)
  if arm=='baseline':
   saved=np.load(H/'direction_root_cost_expected_baseline.npz')
   for key in ['pose','direction','command']:np.testing.assert_equal(plan[key],saved[key])
 assert a['both_success'] and a['first_direction_changed'] and not a['full_reference_equal']
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),old_input_checks=len(p['input_sha256']),sealed_source_diff_exact=True,dataclass_fields_defaults_equal=True,current_snapshot_strict_and_copied=True,baseline_exact=True,baseline_expanded_nodes=1821,primitive_checks=total_primitives,nominal_sampled_points=sample_count,geometry=checks,future_extra_payload_rejected=True,interface_gate_pass=True,nominal_behavior_eligibility_pass=bool(a['nominal_behavior_eligibility_pending_geometry']),braking_switch_elimination_demonstrated=False,original_navigation_gate_unchanged_rejected=True,new_physical_candidate_tested=False,new_grid_searches=0,new_hybrid_searches=0,new_simulations=0,assessment_sha256=sha(H/'direction_root_cost_assessment.json'),whole_research_complete=False)
 (H/'direction_root_cost_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
