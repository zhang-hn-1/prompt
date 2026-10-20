"""Postseal nominal-curve body proxy against scene; never actor map or safety proof."""
from pathlib import Path
import json,hashlib,numpy as np
from execution_domain_four_point_source_v2 import supported_curvature
from supported_four_point_planning import sample_motion_primitive
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1'
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for f,s in seal['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s
cyl=json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text());centres=np.array([v['position'][:2] for v in cyl]);radii=np.array([v['radius'] for v in cyl]);rows=json.loads((D/'policy_records.json').read_text());seen=set();scores=[];records=[]
for i,r in enumerate(rows):
 p=D/'local_plans'/f'frame_{i:03d}.json'
 if not p.exists():continue
 plan=json.loads(p.read_text());v=plan['reference_version']
 if v in seen:continue
 seen.add(v);w=plan['waypoints'];pts=[]
 for a,b in zip(w[:-1],w[1:]):
  k=supported_curvature(a['main_command_rad_s'],a['auxiliary_angle_rad']);assert k is not None
  arc=.3 if a['main_command_rad_s']>0 else -.3
  sample=sample_motion_primitive(start_pose=(a['x_m'],a['y_m'],a['yaw_rad']),signed_arc_length_m=arc,curvature_1_m=k,sample_spacing_m=.01)
  np.testing.assert_allclose(sample[-1,:2],[b['x_m'],b['y_m']],atol=1e-8)
  pts.append(sample[:,:2])
 xy=np.vstack(pts);surface=np.linalg.norm(xy[:,None,:]-centres[None,:,:],axis=2)-radii[None,:]-.4
 j=np.unravel_index(np.argmin(surface),surface.shape);nom=float(surface[j])
 # 1-Lipschitz distance: every point lies <=.005 arc from a sample.
 lower=nom-.005
 scores.append(dict(frame=i,reference_version=v,minimum_sampled_body_proxy_clearance_m=nom,conservative_arc_sampling_lower_bound_m=lower,nearest_obstacle=cyl[j[1]]['path'],at_xy=xy[j[0]].tolist(),proxy_overlap_sampled=nom<0,complete_CAD_safe=False,unknown_is_free=False))
 # Compare SAME nominal sampled centres with frozen actor risk field, without changing it.
 grid=np.load(D/'maps'/f'frame_{i:03d}.npz');cells=np.rint((xy-grid['origin'])/float(grid['resolution'])).astype(int);inside=(cells>=0).all(1)&(cells<grid['labels'].shape).all(1)
 records.append(dict(frame=i,nominal_samples=len(xy),outside_grid=int((~inside).sum()),sample_centres_unknown=int((grid['labels'][tuple(cells[inside].T)]<0).sum())))
z=np.load(D/'physics.npz');q=z['position_world_xy'].astype(float);d=np.linalg.norm(q[:,None,:]-centres[None,:,:],axis=2)-radii[None,:]-.4
result=dict(decision='POSTSEAL_REFERENCE_COVERAGE_DIAGNOSTIC_ONLY',references=len(scores),nominal_references_with_sampled_body_proxy_overlap=sum(s['proxy_overlap_sampled'] for s in scores),min_nominal_proxy_sampled_clearance_m=min(s['minimum_sampled_body_proxy_clearance_m'] for s in scores),actual_sampled_body_proxy_clearance_m=float(d.min()),scores=scores,actor_labels_audit=records,partial_source_precision_does_not_prove_full_obstacle_coverage=True,GT_used_only_after_sealed_decisions=True,no_new_physics_or_fit=True,not_full_CAD_or_safe_stopping_certificate=True,not_actual_collision_claim=True,not_navigation_admission=True,whole_research_complete=False)
(H/'minimum_prefix_reference_GT_coverage_audit_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['scores','actor_labels_audit']},indent=2))
