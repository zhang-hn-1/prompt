"""One factual failure boundary audit; no alternate physical rollout or relaxed gate."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.ndimage import distance_transform_edt
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from execution_domain_four_point_consumer_v2 import planning_clearance
H=Path(__file__).resolve().parent;D=H/'original_E1_tube_viability_navigation_run_v1';O=H/'E1_tube_viability_endpoint_block_cause_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(p)==s,p
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());goal=np.array([3.300000005960465,5.960465010446114e-9]);events=[]
for row in rows:
 if not row['local_plan_created'] or row['plan']['status']!='planning_commitment_blocks_endpoint':continue
 i=row['frame'];q=np.load(D/'maps'/f'frame_{i:03d}.npz');xy=np.asarray(states[i]['position'])[:2];g=StaticSparsePointRiskGrid(q['origin'],10,static_scene_assumption=True);g.labels=q['labels'].copy();g.codes=q['codes'].copy();g.time=float(q['time_s']);clearance=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;sc=g.cells(xy[None])[0];ec=g.cells(goal[None])[0];threshold=planning_clearance(g);commit=row['route_side_commitment'];axis=np.asarray(commit['axis']);origin=np.asarray(commit['axis_start']);normal=np.array([-axis[1],axis[0]]);cells=np.asarray([sc,ec]);coords=g.origin+cells*g.resolution;long=(coords-origin)@axis;side=(coords-origin)@normal;lo,hi=commit['longitudinal_span_m'];prior=(long>=lo)&(long<=hi)&(commit['side']*side<-.5*g.resolution);base=clearance[tuple(cells.T)]>=threshold-1e-8
 route,status=g.plan(xy,goal);assert route is None if not base.all() else True
 events.append(dict(frame=i,time_s=row['time_s'],current_xy=xy.tolist(),start_clearance_m=float(clearance[tuple(sc)]),goal_clearance_m=float(clearance[tuple(ec)]),frozen_planning_clearance_m=threshold,start_base_traversable=bool(base[0]),goal_base_traversable=bool(base[1]),start_prior_excluded=bool(prior[0]),goal_prior_excluded=bool(prior[1]),same_map_no_commitment_plan_status=status,same_map_no_commitment_has_route=route is not None,guard=row['reference_decision']['guard']))
assert events
failed=[r for r in rows if r['plan'].get('status')=='planning_commitment_blocks_endpoint'];successful=[r for r in rows if r['local_plan_created'] and r['plan'].get('status')=='success'];zero=sum(np.array_equal(np.asarray(r['request']),np.zeros(10)) for r in failed)
a=dict(decision='FACTUAL_ENDPOINT_BLOCK_LAYER_IDENTIFIED_ONLY',first_block=events[0],last_block=events[-1],failed_rebuild_attempts=len(events),successful_reference_installations=len(successful),all_base_start_blocked=all(not r['start_base_traversable'] for r in events),all_prior_does_not_exclude_either_endpoint=all(not r['start_prior_excluded'] and not r['goal_prior_excluded'] for r in events),all_same_map_baseline_route_also_blocked=all(not r['same_map_no_commitment_has_route'] for r in events),zero_requests_under_failed_plan=zero,total_action_frames_under_failed_plan=len(failed),next_control_issue='Replanning viability when newly measured hazards consume the frozen planning tracking tube while retained-reference guard uses a different necessary margin. Determine hazard correctness and emergence first; do not reduce clearance or free start cells automatically.',not_proof_of_hidden_collision_or_bad_PID=True,no_new_physics_or_NN=True,no_controller_or_threshold_change=True,no_alternate_future_physical_label=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'events.json').write_text(json.dumps(events,indent=2)+'\n');print(json.dumps({k:v for k,v in a.items() if k not in ['first_block','last_block']},indent=2))
