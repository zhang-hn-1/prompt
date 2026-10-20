"""Older exposure versus computationally delayed latest exposure; no truth."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from causal_source_delivery_memory import CausalSourceDeliveryMemory
from partial_point_risk_grid import PartialPointRiskGrid
from execution_domain_four_point_consumer_v2 import planning_clearance
H=Path(__file__).resolve().parent;D=H/'live_async_stereo_navigation_run_v1';O=H/'live_async_memory_role_v2';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p in ['live_async_memory_consumption_assessment_v2.json','live_async_stereo_policy_causal_assessment_v2.json','live_async_stereo_navigation_source_assessment_v1/assessment.json']:
 assert json.loads((H/p).read_text())['decision'].startswith('PASS')
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s
sources=json.loads((D/'source_rows.json').read_text());policy=json.loads((D/'policy_records.json').read_text());native=json.loads((D/'native_predictions.json').read_text());m=CausalSourceDeliveryMemory();cursor=0;latest=None;rows=[];origin=np.asarray(native[0]['position_world'])[:2]-20;allgrid=PartialPointRiskGrid(origin,10);lastgrid=PartialPointRiskGrid(origin,10)
for i,r in enumerate(policy):
 t=r['time_s'];root=np.asarray(native[i*5]['position_world'])
 while cursor<len(sources) and sources[cursor]['available_frame']<=i:
  s=sources[cursor];z=np.load(D/'partial_observations'/f"frame_{s['frame']:03d}.npz");latest=dict(frame=s['frame'],time_s=s['time_s'],world=z['world'],query_pixel_id=z['query_pixel_id'],partner_pixel_id=z['partner_pixel_id'],input_sha256=s['provenance']);m.deliver(latest,available_time_s=s['available_time_s'],now=s['available_time_s']);cursor+=1
 snap=m.snapshot(t);records=snap['records'];current=[x for x in records if x['frame']==latest['frame']];older=[x for x in records if x['frame']!=latest['frame']]
 for g,rs in ((allgrid,records),(lastgrid,current)):
  g.set_source(dict(frame=snap['source_frame'],time_s=snap['source_time_s'],records=rs));g.update_points(np.empty((0,3)),root,t)
 saved=np.load(D/'maps'/f'frame_{i:03d}.npz');np.testing.assert_array_equal(allgrid.labels,saved['labels']);oldonly=(allgrid.labels==1)&(lastgrid.labels!=1);common=(allgrid.labels==1)&(lastgrid.labels==1);clear=planning_clearance(allgrid)
 A=distance_transform_edt(np.pad(allgrid.labels!=1,1))[1:-1,1:-1]*allgrid.resolution;L=distance_transform_edt(np.pad(lastgrid.labels!=1,1))[1:-1,1:-1]*lastgrid.resolution;extra=(A<clear-1e-8)&(L>=clear-1e-8)
 reference=np.asarray(r.get('installed_reference_xy_after_act',[]));sampled=[]
 if reference.ndim==2 and len(reference)>=2:
  for a,b in zip(reference[:-1],reference[1:]):sampled.extend(a+(b-a)*v for v in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)))
 samples=np.asarray(sampled).reshape(-1,2);cells=allgrid.cells(samples);inside=allgrid.inside(cells);cells=cells[inside];hits=0 if not len(cells) else int(extra[tuple(cells.T)].sum())
 rows.append(dict(frame=i,time_s=t,latest_source_frame=latest['frame'],latest_birth_age_s=t-latest['time_s'],latest_delivered_points=sum(len(x['world']) for x in current),older_exposure_points=sum(len(x['world']) for x in older),shared_hazard_cells=int(common.sum()),older_only_hazard_cells=int(oldonly.sum()),additional_nominal_planning_blocked_cells=int(extra.sum()),current_known_reference_sample_hits_additional_blocked=hits,reference_is_current_plan_not_future_truth=True))
a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='CLOSED_MECHANISM_AUDIT_NOT_CAUSAL_GAIN',preaction_maps_exact=len(rows),latest_delayed_record_epoch_pairs=sum(r['latest_delivered_points'] for r in rows),genuine_older_record_epoch_pairs=sum(r['older_exposure_points'] for r in rows),shared_hazard_cell_time_pairs=sum(r['shared_hazard_cells'] for r in rows),older_only_hazard_cell_time_pairs=sum(r['older_only_hazard_cells'] for r in rows),additional_nominal_blocked_cell_time_pairs=sum(r['additional_nominal_planning_blocked_cells'] for r in rows),current_reference_hits_additional_blocked=sum(r['current_known_reference_sample_hits_additional_blocked'] for r in rows),not_unique_points_or_visibility_or_worldmodel_benefit=True,no_counterfactual_navigation_labels=True,whole_complete=False)
(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'seal.json').write_text(json.dumps(dict(code_sha256=sha(__file__),raw_manifest_sha256=sha(D/'artifact_manifest_post_close.json'),rows_sha256=sha(O/'rows.json'),assessment_sha256=sha(O/'assessment.json')),indent=2)+'\n');print(json.dumps(a,indent=2))
