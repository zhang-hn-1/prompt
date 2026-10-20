"""One historical mechanism probe. Plane is assumed, not measured free space."""
from pathlib import Path
import hashlib,json
import numpy as np,cv2
from paired_current_evidence_v1 import qualify_packet
H=Path(__file__).resolve().parent;D=H/'paired_current_A2_run_v1';O=H/'ground_plane_partner_probe_historical_v1';O.mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();files=[D/'sensor_packets/source_035.npz',D/'online_inputs/frame_035.npz',D/'masks/frame_035.npz',D/'estimated_poses/frame_035.npz'];p=dict(np.load(files[0]));rgb=np.load(files[1]);mask=np.load(files[2]);pose=np.load(files[3]);t=float(p['time_s']);assert all(abs(float(v['time_s'])-t)<1e-9 for v in [rgb,pose]);f=584.8888888888889;cx,cy=319.5,239.5;ids=p['query_pixel_id'];query=np.stack([ids%640,ids//640],axis=1);partner=np.stack([p['partner_pixel_id']%640,p['partner_pixel_id']//640],axis=1);L=pose['left_camera_usd_world'];R=pose['right_camera_usd_world'];Rl=L[:3,:3]@np.diag([1,-1,-1]);Rr=R[:3,:3]@np.diag([1,-1,-1]);rays=np.column_stack([(query[:,0]-cx)/f,(query[:,1]-cy)/f,np.ones(len(ids))])@Rl.T
with np.errstate(divide='ignore',invalid='ignore'):depth=-L[2,3]/rays[:,2];w=L[:3,3]+rays*depth[:,None];loc=(w-R[:3,3])@Rr;uv=loc[:,:2]/loc[:,2,None]*f+[cx,cy]
valid=(depth>0)&(loc[:,2]>0)&np.isfinite(uv).all(1)
off=np.array([(x,y) for y in range(-2,3) for x in range(-2,3)],float)
def sample(im,centres):
 u=centres[:,None,:]+off[None,:,:];inside=np.isfinite(u).all((1,2))&(u[:,:,0]>=0).all(1)&(u[:,:,0]<=639).all(1)&(u[:,:,1]>=0).all(1)&(u[:,:,1]<=479).all(1);gray=cv2.cvtColor(im,cv2.COLOR_RGB2GRAY).astype(np.float32);safe=np.nan_to_num(u,nan=-10,posinf=-10,neginf=-10);out=cv2.remap(gray,safe[:,:,0].astype(np.float32),safe[:,:,1].astype(np.float32),cv2.INTER_LINEAR);return out,inside
left,li=sample(rgb['left_rgb'],query);raw,ri=sample(rgb['right_rgb'],partner);plane,pi=sample(rgb['right_rgb'],uv)
def costs(b):
 a=left-left.mean(1,keepdims=True);c=b-b.mean(1,keepdims=True);rms=np.sqrt(((a-c)**2).mean(1));ham=((left<left[:,12,None])!=(b<b[:,12,None])).sum(1);return rms,ham
nr,nh=costs(raw);pr,ph=costs(plane);q=qualify_packet(p,rgb['left_rgb'],rgb['right_rgb'],mask['left'],mask['right'],t,t,35)
pred=O/'predictions.npz';np.savez_compressed(pred,query_uv=query,NN_partner_uv=partner,prior_partner_uv=uv,prior_world=w,valid_prior=valid,patch_inside=li&ri&pi,NN_rms=nr,prior_rms=pr,NN_census=nh,prior_census=ph,eligible=q['eligible'],time_s=t,ground_is_prior=True,no_source_admission=True)
(O/'prediction_seal.json').write_text(json.dumps(dict(inputs={str(x):sha(x) for x in files},output_sha256=sha(pred),protocol_sha256=sha(H/'ground_plane_partner_probe_protocol_v1.json'),code_sha256=sha(__file__),no_teacher_input=True),indent=2)+'\n')
# Teacher read only after predictions are fixed; no production or threshold change.
g=np.load(D/'offline_truth/frame_035.npz');d=g['left_depth_m'][query[:,1],query[:,0]];G=g['left_camera_usd_world'];Rg=G[:3,:3]@np.diag([1,-1,-1]);true=G[:3,3]+(np.column_stack([(query[:,0]-cx)/f,(query[:,1]-cy)/f,np.ones(len(ids))])*d[:,None])@Rg.T;gv=np.isfinite(d)&(d>0);err=np.linalg.norm(true-p['world'],axis=1);floor=gv&(np.abs(true[:,2])<1e-4);bad=floor&(err>.1)&q['eligible'];good=floor&(err<=.1)&q['eligible'];rows={}
for name,k in [('failed_floor',bad),('correct_floor',good)]:
 selected=k&valid&li&ri&pi;ix=np.flatnonzero(selected);rows[name]=dict(all_points=int(k.sum()),valid_comparable_patches=len(ix),NN_rms_median=float(np.median(nr[ix])) if len(ix) else None,prior_rms_median=float(np.median(pr[ix])) if len(ix) else None,prior_rms_better=int((pr[ix]<nr[ix]).sum()),NN_census_median=float(np.median(nh[ix])) if len(ix) else None,prior_census_median=float(np.median(ph[ix])) if len(ix) else None,prior_census_better=int((ph[ix]<nh[ix]).sum()),prior_world_error_median_m=float(np.median(np.linalg.norm(w[ix]-true[ix],axis=1))) if len(ix) else None)
r=dict(scope='HISTORICAL_POSTSEAL_DESCRIPTIVE_NO_FILTER_OR_GATE',raw_points=len(ids),rows=rows,teacher_read_after_seal=True,prior_is_not_observation=True,no_source_rescoring_or_actor=True,assumption='Only flat floor and estimated camera transforms; geometric error alone cannot prove a match or free space');(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
