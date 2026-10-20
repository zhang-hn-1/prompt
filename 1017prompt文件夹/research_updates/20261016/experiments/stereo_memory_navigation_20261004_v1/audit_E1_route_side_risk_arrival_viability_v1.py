"""Factual causal ordering of exposure arrival and replanning viability, no new policy."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.ndimage import distance_transform_edt
from execution_domain_four_point_consumer_v2 import planning_clearance
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
H=Path(__file__).resolve().parent;D=H/'original_E1_route_side_navigation_resumed_run_v2';O=H/'E1_route_side_risk_arrival_viability_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(p)==s,p
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());events=[]
# Freeze one factual last-successful reference, no new simulations or command labels.
last=next(r for r in reversed(rows) if r['local_plan_created'] and r['plan'].get('status')=='success');start=last['frame'];stop=next(r['frame'] for r in rows[start:] if r['local_plan_created'] and r['plan'].get('status')!='success')
for i in range(start,stop+1,5):
 r=rows[i];q=np.load(D/'maps'/f'frame_{i:03d}.npz');g=StaticSparsePointRiskGrid(q['origin'],10,static_scene_assumption=True);g.labels=q['labels'];c=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;xy=np.asarray(states[i]['position'])[:2];sc=g.cells(xy[None])[0];guard=r.get('reference_decision',{}).get('guard');suffix=None if guard is None else np.asarray(guard['suffix_xy']);minimum=None
 if suffix is not None and len(suffix)>1:
  segments=[np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)) for a,b in zip(suffix[:-1],suffix[1:])];samples=np.vstack(segments);cells=g.cells(samples);assert g.inside(cells).all();minimum=float(c[tuple(cells.T)].min())
 events.append(dict(frame=i,time_s=r['time_s'],current_start_clearance_m=float(c[tuple(sc)]),planning_clearance_m=planning_clearance(g),current_start_replannable=bool(c[tuple(sc)]>=planning_clearance(g)-1e-8),retained_suffix_raster_clearance_m=minimum,retained_suffix_inside_existing_planning_tube=minimum is not None and minimum>=planning_clearance(g)-1e-8,original_guard_retention=bool(guard is not None and guard['retention_necessary_screen_pass']),reference_decision=r['reference_decision']['decision'],source_frame=r['source_frame']))
first_risk=next((r for r in events if r['original_guard_retention'] and not r['retained_suffix_inside_existing_planning_tube']),None);first_block=next((r for r in events if not r['current_start_replannable']),None)
a=dict(decision='FACTUAL_REPLANNING_VIABILITY_ORDER_ONLY',reference_installed_frame=start,first_failed_rebuild_frame=stop,first_retained_suffix_outside_planning_tube=first_risk,first_start_nonreplannable=first_block,earlier_viable_replan_window=first_risk is not None and first_risk['current_start_replannable'] and first_block is not None and first_risk['frame']<first_block['frame'],old_control_reject_preserved=True,no_new_threshold_or_parameter=True,not_control_candidate_admission=True,no_new_physics_or_NN=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'events.json').write_text(json.dumps(events,indent=2)+'\n');print(json.dumps(a,indent=2))
