"""Readonly proposals from past source geometry, now four-view RGB corroboration."""
from pathlib import Path
import cv2, numpy as np, json, hashlib, datetime
from past_point_rgb_memory import PastPointRGBMemory
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'causal_four_view_new_region_v1'
assert not D.exists();D.mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
plan=H.parent.parent/'docs/superpowers/plans/2026-10-07-causal-four-view-new-region-diagnostic.md'
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),plan_sha256=sha(plan),code_sha256=sha(__file__),frozen_memory_sha256=sha(H/'past_point_rgb_memory.py'),knownpose_privileged=True,development_not_fresh=True,maximum_past_epochs=4,window_s=2.,min_both_camera_translation_m=.05,NCC=.8,patch=7,whole_research_complete=False)
(D/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
inputs={};history=[];rows=[]
def load(p):inputs[str(p.resolve())]=sha(p);return np.load(p)
def ncc(a,b):
 a=a.astype(float).ravel();b=b.astype(float).ravel();aa=a-a.mean();bb=b-b.mean();den=np.linalg.norm(aa)*np.linalg.norm(bb)
 return float(aa@bb/den) if den>0 else np.nan
def patch(gray,xy):return cv2.getRectSubPix(gray,(7,7),tuple(np.asarray(xy,float)))
for f in sorted((O/'sensor_packets').glob('*.npz')):
 src=load(f);i=int(src['frame']);t=float(src['time_s']);rgb=load(O/'online_inputs'/f'frame_{i:03d}.npz');pose=load(O/'known_poses'/f'frame_{i:03d}.npz');mask=load(O/'masks'/f'frame_{i:03d}.npz')
 assert float(rgb['time_s'])==float(pose['time_s'])==t
 groups=[]
 for old in history[-4:]:
  oldsrc,oldrgb,oldpose,oldgray=old;st=float(oldsrc['time_s']);assert st<t
  if t-st>2.+1e-9:continue
  displacement=[float(np.linalg.norm(pose[e+'_camera_usd_world'][:3,3]-oldpose[e+'_camera_usd_world'][:3,3])) for e in ['left','right']]
  m=PastPointRGBMemory();m.seed(oldsrc,oldrgb,st,qualified=True);proof=m.query(rgb,pose,mask,t);accepted=proof['appearance_corroborated'].copy();cross_old=np.full(len(accepted),np.nan);cross_now=np.full(len(accepted),np.nan)
  if min(displacement)>=.05:
   gray=[cv2.cvtColor(rgb[k],cv2.COLOR_RGB2GRAY) for k in ['left_rgb','right_rgb']]
   for j in np.flatnonzero(accepted):
    q=int(proof['past_query_id'][j]);p=int(proof['past_partner_id'][j]);a=patch(oldgray[0],[q%640,q//640]);b=patch(oldgray[1],[p%640,p//640]);cross_old[j]=ncc(a,b);a=patch(gray[0],proof['left_uv'][j]);b=patch(gray[1],proof['right_uv'][j]);cross_now[j]=ncc(a,b)
   accepted &= (cross_old>=.8)&(cross_now>=.8)
  else:accepted[:]=False
  name=f'frame_{i:03d}_source_{int(oldsrc["frame"]):03d}.npz'
  np.savez_compressed(D/name,**proof,confirmed_four_view=accepted,cross_eye_source_NCC=cross_old,cross_eye_current_NCC=cross_now,both_camera_displacement_m=displacement,current_frame=i,current_time_s=t)
  groups.append(dict(source_frame=int(oldsrc['frame']),source_time_s=st,temporal_candidates=int(proof['appearance_corroborated'].sum()),confirmed=int(accepted.sum()),both_camera_displacement_m=displacement,file=name))
 history.append((src,rgb,pose,[cv2.cvtColor(rgb[k],cv2.COLOR_RGB2GRAY) for k in ['left_rgb','right_rgb']]))
 rows.append(dict(frame=i,time_s=t,groups=groups,confirmed=sum(x['confirmed'] for x in groups)))
(D/'records.json').write_text(json.dumps(rows,indent=2)+'\n');outputs=[p for p in D.iterdir() if p.is_file()]
(D/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256={str(p.resolve()):sha(p) for p in outputs},all_outputs_before_renderer_labels=True,no_current_or_future_GT_or_actions_read=True,knownpose_privileged=True,oldsource_not_admitted=True,no_free_or_geometry_age_refresh=True,proposal_confirmation_not_control=True),indent=2)+'\n')
print('source-only four-view proposals sealed',len(rows));print([(r['frame'],r['confirmed']) for r in rows if r['confirmed']])
