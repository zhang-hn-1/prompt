from pathlib import Path
import json,hashlib,datetime
import numpy as np,cv2
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from right_temporal_observability import triangulate
from audit_stereo import summarize
from stereo_geometry import FOCAL
cv2.setNumThreads(4)
H=Path(__file__).resolve().parent;D=H/'native_goal_camera_observation_run_v1';O=H/'right_temporal_observability_preflight_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for f,s in manifest.items():assert sha(f)==s,f
rows=json.loads((D/'source_rows.json').read_text());cad=RobotSelfMask();history=[];counts=[];inputs={}
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';files=[D/'source_inputs'/name,D/'partial_class_masks'/name,D/'estimated_poses'/name,D/'history'/name,D/'sensor_packets'/f'source_{i:03d}.npz']
 for f in files:inputs[str(f)]=sha(f)
 rgb=np.load(files[0])['right_rgb'];cls=np.load(files[1])['right'];T=np.load(files[2])['right_camera_usd_world'].copy();T[:3,:3]=T[:3,:3]@np.diag([1,-1,-1]);h=np.load(files[3])['proprio_history'];t=float(np.load(files[4])['time_s']);selfmask=cad.project(joints_from_proprio(h))['right'];positive=cls&~selfmask;gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
 worlds=[];uvs=[];old_uvs=[];old_frames=[];past_count=0;feature_count=0;tracked_count=0;geometric_count=0;baselines=[]
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
  w,g=triangulate(a[ids],past['optical_world'],b[ids],T);geometric_count+=int(g.sum());worlds.append(w[g]);uvs.append(b[ids][g]);old_uvs.append(a[ids][g]);old_frames.append(np.full(int(g.sum()),past['frame'],int))
 world=np.vstack(worlds) if worlds else np.empty((0,3));uv=np.vstack(uvs) if uvs else np.empty((0,2));old_uv=np.vstack(old_uvs) if old_uvs else np.empty((0,2));pf=np.concatenate(old_frames) if old_frames else np.empty(0,int)
 pixels=np.rint(uv).astype(int);unique=len(np.unique(pixels,axis=0));np.savez_compressed(O/name,world=world,current_uv=uv,past_uv=old_uv,past_frame=pf,current_frame=i,current_time_s=t,certified_free=False,not_full_volume=True)
 counts.append(dict(frame=i,time_s=t,past_views=past_count,features=feature_count,tracked=tracked_count,geometric=geometric_count,unique_current_pixels=unique,baseline_min_m=min(baselines) if baselines else None,baseline_max_m=max(baselines) if baselines else None))
 corners=cv2.goodFeaturesToTrack(gray,500,.01,5,mask=positive.astype(np.uint8)*255,blockSize=3)
 if corners is None:corners=np.empty((0,1,2),np.float32)
 history.append(dict(frame=i,time_s=t,optical_world=T.copy(),gray=gray.copy(),corners=corners.copy()))
(O/'attrition.json').write_text(json.dumps(counts,indent=2)+'\n');outputs={str(f):sha(f) for f in O.iterdir()}
source=[Path(__file__),H/'right_temporal_observability.py',H.parent/'clean_visual_hard0_intervention_20261003_v1/temporal_tracks.py',H.parent/'bilateral_temporal_geometry_navigation_20261003_v1/geometry.py']
(O/'pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256=outputs,source_sha256={str(f):sha(f) for f in source},all29=True,current_and_past_only=True,no_network_depth_input=True,no_fixture_shape_filter=True,not_online=True),indent=2)+'\n')
# Independent labels, after complete prediction seal.
es=[];cs=[];scores=[]
for row in counts:
 i=row['frame'];s=np.load(O/f'frame_{i:03d}.npz');truth=np.load(D/'frames'/f'frame_{i:03d}.npz');uv=np.rint(s['current_uv']).astype(int);x,y=uv[:,0],uv[:,1];z=truth['right_depth_m'][y,x];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))));T=truth['right_camera_usd_world'];w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(1);e=np.linalg.norm(s['world']-w,axis=1);e[~valid]=np.inf;c=valid&(np.linalg.norm(w[:,:2],axis=1)<=.351)&(w[:,2]>.001)&(w[:,2]<=1.001);es.append(e);cs.append(c);scores.append(dict(frame=i,n=len(e),unique_current_pixels=row['unique_current_pixels'],metric=summarize(e),class_precision=float(c.mean()) if len(c) else None))
e=np.concatenate(es);c=np.concatenate(cs);m=summarize(e);precision=float(c.mean()) if len(c) else 0.;gates=dict(n=len(e)>=100,metric95=m['within_0p10m'] is not None and m['within_0p10m']>=.95,p90=m['finite_p90_m'] is not None and m['finite_p90_m']<=.1,class95=precision>=.95,two100unique_pixel_exposures=sum(r['unique_current_pixels']>=100 for r in counts)>=2)
r=dict(decision='PASS_DEVELOPMENT_TEMPORAL_GEOMETRY_ONLY' if all(gates.values()) else 'REJECT_RIGHT_TEMPORAL_GEOMETRY',gates=gates,n=len(e),world_metric=m,class_precision=precision,nonempty_exposures=sum(r['geometric']>0 for r in counts),total_features=sum(r['features'] for r in counts),total_tracked=sum(r['tracked'] for r in counts),frames=scores,seen_development=True,not_online_or_navigation=True,older_LK_reject_preserved=True,observer_prior_reject_preserved=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='frames'},indent=2))
