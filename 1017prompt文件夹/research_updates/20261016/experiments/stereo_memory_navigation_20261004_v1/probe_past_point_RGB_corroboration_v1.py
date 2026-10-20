"""Source-only past point -> current RGB appearance; no renderer labels loaded."""
from pathlib import Path
import numpy as np,json,cv2,hashlib,datetime
H=Path(__file__).resolve().parent;O=H/'anchor_validation_fresh_run_v1';OUT=H/'past_point_RGB_corroboration_predictions_v1';assert not OUT.exists();OUT.mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();inputs={};outputs={}
def load(p):inputs[str(p)]=sha(p);return np.load(p)
seed=load(O/'sensor_packets/source_180.npz');rgb0=load(O/'online_inputs/frame_180.npz');haz=(seed['world'][:,2]>.05)&(seed['world'][:,2]<=1.5);world=seed['world'][haz];ids=[seed['query_pixel_id'][haz],seed['partner_pixel_id'][haz]];gray0=[cv2.cvtColor(rgb0[x+'_rgb'],cv2.COLOR_RGB2GRAY) for x in ['left','right']]
for frame in [185,190,240,350]:
 rgb=load(O/'online_inputs'/f'frame_{frame:03d}.npz');pose=load(O/'known_poses'/f'frame_{frame:03d}.npz');mask=load(O/'masks'/f'frame_{frame:03d}.npz');assert float(rgb['time_s'])==float(pose['time_s']) and float(seed['time_s'])<float(rgb['time_s']);ok=np.ones(len(world),bool);scores=np.full((len(world),2),np.nan);std=np.full((len(world),2,2),np.nan);uvs=[];zs=[]
 for eyei,eye in enumerate(['left','right']):
  T=pose[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);local=(world-T[:3,3])@R;uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5];uvs.append(uv);zs.append(local[:,2]);valid=(local[:,2]>0)&np.isfinite(uv).all(axis=1)&(uv[:,0]>=3)&(uv[:,0]<=636)&(uv[:,1]>=3)&(uv[:,1]<=476);current=cv2.cvtColor(rgb[eye+'_rgb'],cv2.COLOR_RGB2GRAY)
  for j in np.flatnonzero(valid):
   x,y=np.rint(uv[j]).astype(int);ox=int(ids[eyei][j]%640);oy=int(ids[eyei][j]//640)
   if mask[eye][y,x] or not(3<=ox<=636 and 3<=oy<=476):valid[j]=False;continue
   a=cv2.getRectSubPix(gray0[eyei],(7,7),(float(ox),float(oy))).astype(float).ravel();b=cv2.getRectSubPix(current,(7,7),tuple(uv[j].astype(float))).astype(float).ravel();std[j,eyei]=[a.std(),b.std()];aa=a-a.mean();bb=b-b.mean();den=np.linalg.norm(aa)*np.linalg.norm(bb);scores[j,eyei]=float(aa@bb/den) if den>0 else np.nan
  ok&=valid
 accepted=ok&np.all(std>=5,axis=(1,2))&np.all(scores>=.8,axis=1);f=OUT/f'frame_{frame:03d}.npz';np.savez_compressed(f,past_world=world,past_query_id=ids[0],past_partner_id=ids[1],source_geometry_time_s=seed['time_s'],current_appearance_time_s=rgb['time_s'],left_uv=uvs[0],right_uv=uvs[1],left_z=zs[0],right_z=zs[1],bilateral_fov_and_mask_valid=ok,patch_std=std,patch_NCC=scores,appearance_corroborated=accepted);outputs[str(f)]=sha(f)
(OUT/'all_predictions_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,prediction_sha256=outputs,source_code_sha256=sha(__file__),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-07-past-point-RGB-corroboration.md'),renderer_labels_not_loaded=True,privileged_current_knownpose=True,original_geometry_age_not_refreshed=True,development_not_fresh=True),indent=2)+'\n')
print('All four RGB-only outputs sealed',[(p.name,int(np.load(p)['appearance_corroborated'].sum())) for p in OUT.glob('*.npz')])
