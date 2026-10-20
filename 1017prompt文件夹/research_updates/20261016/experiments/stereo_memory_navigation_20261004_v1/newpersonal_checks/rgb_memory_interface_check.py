from pathlib import Path
import sys,json,hashlib,datetime,copy
import numpy as np
H=Path(__file__).resolve().parents[1];sys.path.insert(0,str(H))
from past_point_rgb_memory import PastPointRGBMemory
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
checks=Path(__file__).resolve().parent
# Existing bytes tracked without changing any author artifact.
protected=[H/'past_point_rgb_memory.py',H/'test_past_point_rgb_memory.py',H/'online_RGB_memory_protocol_v1.json',H/'online_RGB_memory_fresh_assessment_v1.json',H/'online_RGB_memory_prelabel_recompute_v1.json',H/'past_point_RGB_corroboration_assessment_v1.json']
before={str(p):sha(p) for p in protected}
old=H/'anchor_validation_fresh_run_v1';oldpred=H/'past_point_RGB_corroboration_predictions_v1';m=PastPointRGBMemory();m.seed(np.load(old/'sensor_packets/source_180.npz'),np.load(old/'online_inputs/frame_180.npz'),18.,qualified=True)
oldfields=0
for i in (185,190,240,350):
 r=np.load(old/'online_inputs'/f'frame_{i:03d}.npz');t=float(r['time_s']);d=m.query(r,np.load(old/'known_poses'/f'frame_{i:03d}.npz'),np.load(old/'masks'/f'frame_{i:03d}.npz'),t);s=np.load(oldpred/f'frame_{i:03d}.npz')
 for k in s.files:np.testing.assert_equal(s[k],d[k]);oldfields+=1
O=H/'online_RGB_memory_fresh_run_v1';states=json.loads((O/'states.json').read_text());records=json.loads((O/'rgb_memory_records.json').read_text());sealed=json.loads((O/'artifact_manifest_post_close.json').read_text());sealchecks=0
for f,s in sealed['artifact_sha256'].items():assert sha(f)==s,f;sealchecks+=1
m=PastPointRGBMemory();outputs=[];items=[];fields=0;seed=None
class Bomb(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future payload accessed')
  return super().__getitem__(k)
for rec in records:
 i=rec['frame'];t=rec['time_s'];src=np.load(O/'sensor_packets'/f'source_{i:03d}.npz');rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz');mask=np.load(O/'masks'/f'frame_{i:03d}.npz')
 assert int(src['frame'])==i and float(src['time_s'])==t and float(mask['time_s'])==t
 lastfive=states[i-4:i+1] if i>=4 else [];haz=((src['world'][:,2]>.05)&(src['world'][:,2]<=1.5)).sum();q=(18-1e-6<=t<24-1e-6 and len(lastfive)==5 and len({s['time_s'] for s in lastfive})==5 and all(np.linalg.norm(s['velocity'][:2])<.05 for s in lastfive) and haz>=100 and 0<=t-float(src['time_s'])<=.5)
 assert q==rec['qualified_seed']
 if not m.seeded and q:m.seed(src,rgb,t,qualified=True);seed=i
 d=m.query(rgb,pose,mask,t);saved=np.load(O/'rgb_memory_predictions'/f'frame_{i:03d}.npz')
 for k,v in d.items():np.testing.assert_equal(v,saved[k]);fields+=1
 for r,p in [(Bomb(time_s=t+1),pose),(rgb,Bomb(time_s=t+1))]:
  try:m.query(r,p,mask,t)
  except ValueError:pass
  else:raise AssertionError('future accepted')
 dup=m.query(rgb,pose,mask,t)
 for k,v in d.items():np.testing.assert_equal(v,dup[k])
 outputs.append(d);items.append((src,rgb,pose,mask,t,q))
# Independent extended causal prefix includes seed + nonempty 185/190/195 outputs.
prefix=PastPointRGBMemory()
for item,d in zip(items[:40],outputs[:40]):
 src,rgb,pose,mask,t,q=item
 if not prefix.seeded and q:prefix.seed(src,rgb,t,qualified=True)
 x=prefix.query(rgb,pose,mask,t)
 for k,v in d.items():np.testing.assert_equal(v,x[k])
# This is input-local poison isolation; no unread future archive files are consulted.
seedbomb=PastPointRGBMemory()
try:seedbomb.seed(Bomb(time_s=2),Bomb(time_s=2),1.,qualified=True)
except ValueError:pass
else:raise AssertionError('future source accepted')
# Labels read only after independent reconstruction/prefix/poison checks above.
rows=[]
for rec,d in zip(records,outputs):
 sel=d['appearance_corroborated'];pts=d['past_world'][sel];errs=[];i=rec['frame'];gt=np.load(O/'offline_truth'/f'frame_{i:03d}.npz')
 for eye in ('left','right'):
  u=np.rint(d[eye+'_uv'][sel]).astype(int);x,y=u[:,0],u[:,1];z=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);rays=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@R.T;errs.append(np.linalg.norm(actual-pts,axis=1))
 e=np.array(errs).T;finite=np.isfinite(e).all(axis=1);correct=finite&np.all(e<=.1,axis=1);n=int(sel.sum());bins=len(np.unique(np.floor(pts[:,:2]/.1).astype(int),axis=0));src=items[len(rows)][0];haz=int(((src['world'][:,2]>.05)&(src['world'][:,2]<=1.5)).sum());speed=float(np.linalg.norm(states[i]['velocity'][:2]));row=dict(frame=i,accepted=n,both_eye_correct=int(correct.sum()),finite_wrong=int((finite&~correct).sum()),label_insufficient=int((~finite).sum()),precision=float(correct.sum()/n) if n else None,bins=bins,hazards=haz,speed=speed);rows.append(row)
 assert bins==rec['unique_canonical_10cm_XY_bins'] and haz==rec['hazards'] and speed==rec['current_XY_speed_m_s']
 assert bool(d['geometry_expired'])==(m.seeded and rec['time_s']-m.geometry_time>10)
 assert not bool(d['certified_free']) and not bool(d['source_model_admitted'])
 if rec['seeded']:assert d['source_geometry_time_s']==m.geometry_time and d['source_frame']==seed
valid=[r for r in rows if r['accepted']];moving=[r for r in valid if r['speed']>=.05 and r['hazards']==0 and r['accepted']>=20 and r['bins']>=2]
author=json.loads((H/'online_RGB_memory_fresh_assessment_v1.json').read_text())
for a,r in zip(author['rows'],rows):
 for k in ('frame','accepted','both_eye_correct','finite_wrong','label_insufficient','precision'):assert a[k]==r[k]
assert len(author['rows'])==len(rows)==71
assert before=={str(p):sha(p) for p in protected}
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_exact_fields=oldfields,fresh_exact_fields=fields,seed_frame=seed,postclose_hashes_checked=sealchecks,future_RGB_and_pose_Bomb_rejections=142,future_seed_source_Bomb_rejected=True,duplicate_replays=71,extended_prefix_source_updates=40,extended_prefix_last_frame=195,nonempty_rows=valid,moving_gate_frames=[r['frame'] for r in moving],all_nonempty_precision_ge95=bool(valid) and all(r['precision']>=.95 for r in valid),author_label_rows_exact=True,protected_author_hashes=before,protected_author_bytes_unchanged=True,not_physics_replay=True)
(checks/'rgb_memory_interface_results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
