"""Causal all-output reconstruction BEFORE any fresh renderer labels."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from past_point_rgb_memory import PastPointRGBMemory
H=Path(__file__).resolve().parent;O=H/'online_RGB_memory_fresh_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();status=json.loads((H/'online_RGB_memory_status_v1.json').read_text());assert status['status']=='finished' and status['exit_code']==0
collection=json.loads((H/'online_RGB_memory_collection_verification_v1.json').read_text());assert collection['collection_completed'] and collection['actual_fixed350_requests_and1750_dispatch_ticks_exact']
seal=json.loads((O/'artifact_manifest_post_close.json').read_text());assert all(sha(f)==s for f,s in seal['artifact_sha256'].items());states=json.loads((O/'states.json').read_text());records=json.loads((O/'rgb_memory_records.json').read_text());m=PastPointRGBMemory();first=None;fields=0;replay=[];inputs=[]
class Bomb(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future payload read')
  return super().__getitem__(k)
for r in records:
 i=r['frame'];t=r['time_s'];src=np.load(O/'sensor_packets'/f'source_{i:03d}.npz');rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz');mask=np.load(O/'masks'/f'frame_{i:03d}.npz');q=18-1e-6<=t<24-1e-6 and i>=4 and all(np.linalg.norm(s['velocity'][:2])<.05 for s in states[i-4:i+1]) and int(((src['world'][:,2]>.05)&(src['world'][:,2]<=1.5)).sum())>=100;assert q==r['qualified_seed']
 if not m.seeded and q:m.seed(src,rgb,t,qualified=True);first=i
 d=m.query(rgb,pose,mask,t);saved=np.load(O/'rgb_memory_predictions'/f'frame_{i:03d}.npz')
 for k,v in d.items():np.testing.assert_equal(v,saved[k]);fields+=1
 try:m.query(Bomb(time_s=t+1),pose,mask,t)
 except ValueError:pass
 else:raise AssertionError('future RGB accepted')
 dup=m.query(rgb,pose,mask,t)
 for k,v in d.items():np.testing.assert_equal(v,dup[k])
 inputs.append((src,rgb,pose,mask,t,q));replay.append(d)
# Full-frame-log vs truncation: new state consumes only the first37 historical sourceupdates.
short=PastPointRGBMemory();truncated=0
for item,full in zip(inputs[:37],replay[:37]):
 src,rgb,pose,mask,t,q=item
 if not short.seeded and q:short.seed(src,rgb,t,qualified=True)
 d=short.query(rgb,pose,mask,t)
 for k,v in full.items():np.testing.assert_equal(v,d[k])
 truncated+=1
pre=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),all_source_updates=len(records),exact_saved_fields=fields,first_seed_frame=first,future_payload_poison_tests=len(records),duplicate_replays_no_source_refresh=len(records),truncated_prefix_outputs_exact=truncated,GT_not_loaded_yet=True,postclose_seal_sha256=sha(O/'artifact_manifest_post_close.json'))
(H/'online_RGB_memory_prelabel_recompute_v1.json').write_text(json.dumps(pre,indent=2)+'\n')
rows=[]
for r,d in zip(records,replay):
 i=r['frame'];sel=d['appearance_corroborated'];point=d['past_world'][sel];truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');errors=[]
 for eye in ['left','right']:
  uv=d[eye+'_uv'][sel];xx,yy=np.rint(uv).astype(int).T;z=truth[eye+'_depth_m'][yy,xx];T=truth[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(xx))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@R.T;errors.append(np.linalg.norm(actual-point,axis=1))
 e=np.asarray(errors).T;finite=np.isfinite(e).all(axis=1);good=finite&np.all(e<=.1,axis=1);n=int(sel.sum());rows.append(dict(**r,accepted=n,both_eye_correct=int(good.sum()),finite_wrong=int((finite&~good).sum()),label_insufficient=int((~finite).sum()),precision=float(good.sum()/n) if n else None))
valid=[r for r in rows if r['accepted']];moving=[r for r in rows if r['current_XY_speed_m_s']>=.05 and r['hazards']==0 and r['accepted']>=20 and r['unique_canonical_10cm_XY_bins']>=2]
gates=dict(qualified_seed_present=first is not None,causal_all_outputs_exact=True,future_poison_and_truncation_pass=True,geometry_timestamps_TTL_original=all(not r['geometry_expired'] or r['accepted']==0 for r in rows),moving_depth_gap_20pixels_2bins=bool(moving),all_nonempty_bilateral_precision_ge95=bool(valid) and all(r['precision']>=.95 for r in valid))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='ONLINE_READONLY_APPEARANCE_DIAGNOSTIC_PASS_ONLY' if all(gates.values()) else 'REJECT',gates=gates,prelabel=pre,rows=rows,moving_gap_covered_frames=[r['frame'] for r in moving],knownpose_privileged=True,same_scene_not_unseen_generalization=True,fresh_whole_sequence_after_design=True,no_new_depth_geometry_age_refresh_or_free_certificate=True,no_map_or_action_consumer=True,no_navigation_claim=True)
(H/'online_RGB_memory_fresh_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
