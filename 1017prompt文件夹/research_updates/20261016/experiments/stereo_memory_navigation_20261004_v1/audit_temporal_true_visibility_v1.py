"""Classify TRUE patch visibility only offline; never a source gate."""
from pathlib import Path
import numpy as np,json,cv2,hashlib,datetime
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'temporal_ray_sweep_diagnostic_v1';a=json.loads((D/'assessment_v1.json').read_text());rows=[];dy,dx=np.mgrid[-3:4,-3:4]
for r in a['rows']:
 f=np.load(D/r['file']);uv=f['current_uv'];x,y=uv.astype(int).T;eye=r['eye'];i=r['frame'];truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');puv=uv[:,None,:]+np.stack([dx,dy],axis=-1).reshape(1,49,2);px,py=puv.astype(int).transpose(2,0,1);z=truth[eye+'_depth_m'][py,px];A=truth[eye+'_camera_usd_world'];R=A[:3,:3]@np.diag([1,-1,-1]);ray=np.stack([(puv[:,:,0]-319.5)/584.8888888888889,(puv[:,:,1]-239.5)/584.8888888888889,np.ones(puv.shape[:2])],axis=-1)
 with np.errstate(invalid='ignore'):world=A[:3,3]+(ray*z[:,:,None])@R.T;centre_error=np.linalg.norm(world[:,24]-f['world'],axis=1)
 wrong=np.isfinite(centre_error)&(centre_error>.1);correct=np.isfinite(centre_error)&(centre_error<=.1);patchtruth=np.isfinite(z).all(1);stats=[]
 for j in f['source_frames']:
  old=np.load(O/'offline_truth'/f'frame_{j:03d}.npz');mask=np.load(O/'masks'/f'frame_{j:03d}.npz')[eye];B=old[eye+'_camera_usd_world'];Q=B[:3,:3]@np.diag([1,-1,-1]);loc=(world-B[:3,3])@Q
  with np.errstate(invalid='ignore',divide='ignore'):su=loc[:,:,:2]/loc[:,:,2,None]*584.8888888888889+[319.5,239.5]
  fov=np.isfinite(su).all(2)&(loc[:,:,2]>0)&(su[:,:,0]>=0)&(su[:,:,0]<=639)&(su[:,:,1]>=0)&(su[:,:,1]<=479);su=np.nan_to_num(su,nan=-1000,posinf=-1000,neginf=-1000);free=cv2.remap((~mask).astype(np.float32),su[:,:,0].astype(np.float32),su[:,:,1].astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)>.999
  cx,cy=np.rint(su[:,24]).astype(int).T;inside=fov[:,24];q=np.flatnonzero(inside);seen=np.zeros(len(uv),bool);labelled=np.zeros(len(uv),bool);oz=old[eye+'_depth_m'][cy[q],cx[q]];orr=np.c_[(cx[q]-319.5)/584.8888888888889,(cy[q]-239.5)/584.8888888888889,np.ones(len(q))]
  with np.errstate(invalid='ignore'):ow=B[:3,3]+(orr*oz[:,None])@Q.T;err=np.linalg.norm(ow-world[q,24],axis=1)
  labelled[q]=np.isfinite(err);seen[q]=np.isfinite(err)&(err<=.1);stats.append(dict(source_frame=int(j),current_wrong=int(wrong.sum()),current_wrong_patch_GT_insufficient=int((wrong&~patchtruth).sum()),current_wrong_true_centre_outside_past_FOV=int((wrong&~inside).sum()),current_wrong_true_centre_CAD_blocked=int((wrong&inside&~free[:,24]).sum()),current_wrong_true_centre_renderer_label_insufficient=int((wrong&inside&~labelled).sum()),current_wrong_true_centre_not_same_surface=int((wrong&inside&labelled&~seen).sum()),current_wrong_true_centre_same_surface=int((wrong&seen&free[:,24]).sum()),current_wrong_true_whole_patch_geometric_support=int((wrong&patchtruth&(fov&free).all(1)).sum()),current_correct_true_whole_patch_geometric_support=int((correct&patchtruth&(fov&free).all(1)).sum())))
 rows.append(dict(frame=i,eye=eye,views=stats))
f=D/'true_visibility_attribution_v1.json';assert not f.exists();f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),GT_ONLY_OFFLINE=True,overlapping_counts_not_sum_categories=True,no_source_admission_or_filtered_rescore=True),indent=2)+'\n');print(rows)
