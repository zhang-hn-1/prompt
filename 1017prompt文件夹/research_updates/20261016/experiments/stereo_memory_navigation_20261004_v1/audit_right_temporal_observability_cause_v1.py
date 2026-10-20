from pathlib import Path
import json,hashlib,datetime
import numpy as np,cv2
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from right_temporal_observability import triangulate
from audit_stereo import summarize
from stereo_geometry import FOCAL
cv2.setNumThreads(4)
H=Path(__file__).resolve().parent;D=H/'native_goal_camera_observation_run_v1';O=H/'right_temporal_observability_cause_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for f,s in manifest.items():assert sha(f)==s,f
rows=json.loads((D/'source_rows.json').read_text());cad=RobotSelfMask();history=[];counts=[];inputs={}
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';files=[D/'source_inputs'/name,D/'partial_class_masks'/name,D/'estimated_poses'/name,D/'history'/name,D/'sensor_packets'/f'source_{i:03d}.npz']
 for f in files:inputs[str(f)]=sha(f)
 rgb=np.load(files[0])['right_rgb'];cls=np.load(files[1])['right'];T=np.load(files[2])['right_camera_usd_world'].copy();T[:3,:3]=T[:3,:3]@np.diag([1,-1,-1]);h=np.load(files[3])['proprio_history'];t=float(np.load(files[4])['time_s']);selfmask=cad.project(joints_from_proprio(h))['right'];positive=cls&~selfmask;gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
 diagnostics=[];worlds=[];uvs=[];old_uvs=[];old_frames=[];past_count=0;feature_count=0;tracked_count=0;geometric_count=0;baselines=[]
 for past in history:
  # Independent metadata excludes future payload before image/pose access.
  if past['frame']>=i or past['time_s']>=t:continue
  baseline=np.linalg.norm(T[:3,3]-past['optical_world'][:3,3])
  if baseline<.1:continue
  past_count+=1;baselines.append(float(baseline));corners=past['corners'];feature_count+=len(corners)
  if not len(corners):continue
  forward,ok,_=cv2.calcOpticalFlowPyrLK(past['gray'],gray,corners,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01))
  back,bok,_=cv2.calcOpticalFlowPyrLK(gray,past['gray'],forward,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01))
  a=corners[:,0];b=forward[:,0];valid=(ok[:,0]>0)&(bok[:,0]>0)&np.isfinite(b).all(1)&(np.linalg.norm(back[:,0]-a,axis=1)<=1.5)&(b[:,0]>=0)&(b[:,0]<=639)&(b[:,1]>=0)&(b[:,1]<=479)
  ids=np.flatnonzero(valid);xy=np.rint(b[ids]).astype(int);ids=ids[positive[xy[:,1],xy[:,0]]];tracked_count+=len(ids)
  w,g=triangulate(a[ids],past['optical_world'],b[ids],T);diagnostics.append(dict(past_frame=past['frame'],uv_past=a[ids].tolist(),uv_current=b[ids].tolist(),native_accepted=g.tolist()));geometric_count+=int(g.sum());worlds.append(w[g]);uvs.append(b[ids][g]);old_uvs.append(a[ids][g]);old_frames.append(np.full(int(g.sum()),past['frame'],int))
 world=np.vstack(worlds) if worlds else np.empty((0,3));uv=np.vstack(uvs) if uvs else np.empty((0,2));old_uv=np.vstack(old_uvs) if old_uvs else np.empty((0,2));pf=np.concatenate(old_frames) if old_frames else np.empty(0,int)
 pixels=np.rint(uv).astype(int);unique=len(np.unique(pixels,axis=0));np.savez_compressed(O/name,world=world,current_uv=uv,past_uv=old_uv,past_frame=pf,current_frame=i,current_time_s=t,certified_free=False,not_full_volume=True)
 (O/f'pair_proofs_{i:03d}.json').write_text(json.dumps(diagnostics,indent=2)+'\n');counts.append(dict(frame=i,time_s=t,past_views=past_count,features=feature_count,tracked=tracked_count,geometric=geometric_count,unique_current_pixels=unique,baseline_min_m=min(baselines) if baselines else None,baseline_max_m=max(baselines) if baselines else None))
 corners=cv2.goodFeaturesToTrack(gray,500,.01,5,mask=positive.astype(np.uint8)*255,blockSize=3)
 if corners is None:corners=np.empty((0,1,2),np.float32)
 history.append(dict(frame=i,time_s=t,optical_world=T.copy(),gray=gray.copy(),corners=corners.copy()))
(O/'attrition.json').write_text(json.dumps(counts,indent=2)+'\n');outputs={str(f):sha(f) for f in O.iterdir()}
source=[Path(__file__),H/'right_temporal_observability.py',H.parent/'clean_visual_hard0_intervention_20261003_v1/temporal_tracks.py',H.parent/'bilateral_temporal_geometry_navigation_20261003_v1/geometry.py']
(O/'pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256=outputs,source_sha256={str(f):sha(f) for f in source},all29=True,current_and_past_only=True,no_network_depth_input=True,no_fixture_shape_filter=True,not_online=True),indent=2)+'\n')

old=json.loads((H/'right_temporal_observability_preflight_v1/attrition.json').read_text());assert old==counts
# Reconstructed same78candidate tracks sealed above; GT only cause diagnosis.
scored=[];n=0;true_accepted=0;correct_pairs=0;pixel_errors=[];world_pair_errors=[]
for row in counts:
 i=row['frame'];current=np.load(D/'frames'/f'frame_{i:03d}.npz')
 for pair in json.loads((O/f'pair_proofs_{i:03d}.json').read_text()):
  j=pair['past_frame'];a=np.array(pair['uv_past'],float).reshape(-1,2);b=np.array(pair['uv_current'],float).reshape(-1,2)
  if not len(a):continue
  past=np.load(D/'frames'/f'frame_{j:03d}.npz');A=past['right_camera_usd_world'].copy();B=current['right_camera_usd_world'].copy();A[:3,:3]=A[:3,:3]@np.diag([1,-1,-1]);B[:3,:3]=B[:3,:3]@np.diag([1,-1,-1]);w,ok=triangulate(a,A,b,B);n+=len(a);true_accepted+=int(ok.sum())
  def actual_world(uv,frame,T):
   xy=np.rint(uv).astype(int);z=frame['right_depth_m'][xy[:,1],xy[:,0]];r=np.c_[(uv-[319.5,239.5])/FOCAL,np.ones(len(uv))];return T[:3,3]+(r*z[:,None])@T[:3,:3].T
  wa=actual_world(a,past,A);wb=actual_world(b,current,B);err=np.linalg.norm(wa-wb,axis=1);local=(wa-B[:3,3])@B[:3,:3];proj=local[:,:2]/local[:,2:3]*FOCAL+[319.5,239.5];pix=np.linalg.norm(proj-b,axis=1);correct=np.isfinite(err)&(err<=.1);correct_pairs+=int(correct.sum());world_pair_errors.extend(err.tolist());pixel_errors.extend(pix.tolist());scored.append(dict(current=i,past=j,n=len(a),GT_pose_accepted=int(ok.sum()),world_pair_within10cm=int(correct.sum())))
r=dict(candidate_tracks=n,native_pose_accepted=0,GT_pose_accepted=true_accepted,world_pair_within10cm=correct_pairs,world_pair_metric=summarize(np.array(world_pair_errors)),reprojection_pixel_median=float(np.median(pixel_errors)) if pixel_errors else None,reprojection_pixel_p90=float(np.percentile(pixel_errors,90)) if pixel_errors else None,frames=scored,only_posthoc_cause=True,no_new_candidate_or_gate=True,whole_research_complete=False)
(O/'cause_assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='frames'},indent=2))
