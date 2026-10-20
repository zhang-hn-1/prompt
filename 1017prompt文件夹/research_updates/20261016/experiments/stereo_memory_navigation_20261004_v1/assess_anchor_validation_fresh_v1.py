"""Whole-sequence evaluation only after post-close all-output seal."""
from pathlib import Path
import hashlib,json,datetime,numpy as np
from past_geometry_anchor_filter import PastGeometryAnchorFilter
from packet_scalars import normalize_npz_packet
from anchor_validation_schedule import request
H=Path(__file__).resolve().parent;run=H/'anchor_validation_fresh_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();status=json.loads((H/'anchor_validation_status_v1.json').read_text());assert status['status']=='finished' and status['exit_code']==0
seal=json.loads((run/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==s for p,s in seal['artifact_sha256'].items())
states=json.loads((run/'states.json').read_text());records=json.loads((run/'policy_records.json').read_text());anchors=json.loads((run/'anchor_records.json').read_text());assert len(records)==350 and len(states)==351;clock=np.array([x['time_s'] for x in states]);assert (np.diff(clock)>0).all()
for r in records:np.testing.assert_array_equal(np.array(r['local']['request'],np.float32),request(r['local']['time_s']))
a={r['frame']:r for r in anchors};f=PastGeometryAnchorFilter();rows=[];first=None
# Verify all causal outputs without reading ANY label first.
for p in sorted((run/'sensor_packets').glob('source_*.npz')):
 src=normalize_npz_packet(dict(np.load(p)));i=int(src['frame']);t=float(src['time_s']);q=18-1e-6<=t<24-1e-6 and all(np.linalg.norm(s['velocity'][:2])<.05 for s in states[i-4:i+1]) and int(((src['world'][:,2]>.05)&(src['world'][:,2]<=1.5)).sum())>=100
 keep=f.ingest(src,t,q);saved=np.load(run/'anchor_predictions'/f'frame_{i:03d}.npz');np.testing.assert_array_equal(keep,saved['keep']);assert q==a[i]['qualified_seed']
 if first is None and f.seeded:first=i
(H/'anchor_fresh_prelabel_source_recompute_v1.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),all71_outputs_exact=True,actual_requests350_exact=True,first_seed_frame=first,post_close_seal_SHA256=sha(run/'artifact_manifest_post_close.json')),indent=2)+'\n')
for p in sorted((run/'anchor_predictions').glob('frame_*.npz')):
 pred=np.load(p);i=int(pred['frame'])
 if first is None or i<first:continue
 src=np.load(run/'sensor_packets'/f'source_{i:03d}.npz');truth=np.load(run/'offline_truth'/f'frame_{i:03d}.npz');q=src['query_pixel_id'].astype(int);yy=q//640;xx=q%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]
 with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(src['world']-actual,axis=1)
 hazard=(src['world'][:,2]>.05)&(src['world'][:,2]<=1.5);keep=pred['keep'];good=np.isfinite(err)&(err<=.1);assert not (keep&~hazard).any()
 rows.append(dict(frame=i,time_s=float(src['time_s']),baseline_hazards=int(hazard.sum()),baseline_correct=int((hazard&good).sum()),retained=int(keep.sum()),retained_correct=int((keep&good).sum()),retained_insufficient=int((keep&~np.isfinite(err)).sum()),retained_proven_error=int((keep&np.isfinite(err)&~good).sum())))
def metrics(rs):
 keys=['baseline_hazards','baseline_correct','retained','retained_correct','retained_insufficient','retained_proven_error'];m={k:sum(r[k] for r in rs) for k in keys};m.update(precision=m['retained_correct']/m['retained'] if m['retained'] else None,correct_retention=m['retained_correct']/m['baseline_correct'] if m['baseline_correct'] else None,source_frames=len(rs));return m
m=metrics(rows);eligible=first is not None and len(rows)>=10;passed=eligible and m['precision'] is not None and m['correct_retention'] is not None and m['precision']>=.95 and m['correct_retention']>=.8
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='SOURCE_FILTER_LOCAL_PASS_ONLY' if passed else 'REJECT' if eligible else 'INSUFFICIENT',metrics=m,moving=metrics([r for r in rows if r['time_s']<24-1e-6]),stop=metrics([r for r in rows if r['time_s']>=24-1e-6]),seed_frame=first,rows=rows,knownpose_privileged=True,shared_familiar_seed_prefix=True,not_fresh_scene=True,no_navigation_claim=True,not_observed_free_or_universal_depth_certificate=True,criteria_frozen_before_collection=True)
(H/'anchor_validation_fresh_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
