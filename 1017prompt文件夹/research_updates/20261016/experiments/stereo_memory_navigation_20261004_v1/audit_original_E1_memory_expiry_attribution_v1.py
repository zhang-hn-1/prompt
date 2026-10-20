"""One past-only cumulative sparse hazard shadow at factual states; no control run."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
from partial_point_risk_grid import PartialPointRiskGrid
from execution_domain_four_point_consumer_v2 import local_curve_for_direction_tube,planning_clearance
from prior_rgb_navigation import PriorRGBPolicy
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent;D=H/'original_E1_direct_memory_navigation_run_v1';O=H/'original_E1_memory_expiry_attribution_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for p,d in seal['artifact_sha256'].items():assert sha(p)==d,p
r=json.loads((D/'policy_records.json').read_text());s=json.loads((D/'source_rows.json').read_text());states=json.loads((D/'states.json').read_text());origin=np.array([-3.2999999940395353,5.960465010446114e-9])-20;g=PartialPointRiskGrid(origin,10);clock=np.full(g.labels.shape,-np.inf);cursor=0;rows=[];events=[];protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));drive=PriorRGBPolicy(protocol,'current').tracker.drive_model
for i,row in enumerate(r):
 t=row['time_s']
 while cursor<len(s) and s[cursor]['frame']<=i:
  source=s[cursor];assert source['time_s']<=t;p=np.load(D/'partial_observations'/f"frame_{source['frame']:03d}.npz")['world'];cells=g.cells(p[:,:2]);cells=cells[g.inside(cells)]
  if len(cells):np.maximum.at(clock,tuple(cells.T),source['time_s'])
  cursor+=1
 factual=np.load(D/'maps'/f'frame_{i:03d}.npz')['labels'];union=np.where(np.isfinite(clock),1,-1).astype(np.int8);expired=(union==1)&(factual!=1);rows.append(dict(frame=i,time_s=t,current_cells=int((factual==1).sum()),cumulative_past_observed_cells=int((union==1).sum()),expired_only_cells=int(expired.sum()),static_assumption_shadow_not_live_action=True))
 if row['local_plan_created']:
  xy=np.asarray(states[i]['position'])[:2];R=Rotation.from_quat(np.asarray(states[i]['quaternion'])[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));initial=row['plan']['initial_direction'];pair=[]
  for name,labels in [('factual_TTL10',factual),('cumulative_static_past_only_shadow',union)]:
   g.labels=labels;plan,meta=local_curve_for_direction_tube(g,xy,yaw,np.array([3.300000005960465,5.960465010446114e-9]),drive,initial_direction=initial);pair.append(dict(name=name,status=meta['status'],target=meta['local_target'],path=None if plan is None else [[w.x_m,w.y_m,w.drive_direction] for w in plan.waypoints],direction_changes=None if plan is None else plan.direction_changes,not_alternate_actual_trajectory=True))
  current_actual=json.loads((D/'local_plans'/f'frame_{i:03d}.json').read_text());assert pair[0]['status']==current_actual['status'];np.testing.assert_array_equal(np.asarray([v[:2] for v in pair[0]['path']]),np.asarray(row['installed_reference_xy_after_act']))
  events.append(dict(frame=i,time_s=t,expired_cells=rows[-1]['expired_only_cells'],plans=pair,path_changed=pair[0]['path']!=pair[1]['path']))
a=dict(decision='PAST_ONLY_STATIC_SPARSE_MEMORY_SHADOW_DIAGNOSTIC_ONLY',factual_actions=len(r),events=len(events),events_changed=sum(e['path_changed'] for e in events),expired_cell_time=sum(x['expired_only_cells'] for x in rows),peak_expired_cells=max(x['expired_only_cells'] for x in rows),no_future_source_or_fullscene_actor=True,factual_TTL10_unchanged=True,no_nav_or_safety_gain_claim=True,full60_incomplete=True,whole_research_complete=False)
for name,data in [('assessment',a),('cell_time_rows',rows),('five_installation_shadows',events)]:(O/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(a,indent=2))
