"""Feature-only reader and separate offline teacher reader; no heldout teacher in features."""
from pathlib import Path
import json,hashlib,numpy as np
from stereo_geometry import restore_native
from learned_memory_risk_features_v1 import build_features
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify_sequence(D):
 D=Path(D);m=json.loads((D/'artifact_manifest_post_close.json').read_text())
 if 'status' in m:
  assert m['status']['status']=='finished' and m['status']['child_returncode']==0
 else:
  legacy={'sgbm_baseline35_e1_coverage_run_v1':('sgbm_baseline35_remaining_status_v1.json','e1'),'sgbm_baseline35_e2_run_v1':('sgbm_baseline35_e2_status_v1.json',None)}
  assert D.name in legacy,'only two frozen historical manifest schemas supported'
  filename,condition=legacy[D.name];status=json.loads((D.parent/filename).read_text());assert status['status']=='finished' and status['exit_code']==0
  close=next(row for row in status['sequences'] if row['condition']==condition) if condition else status
  assert close['normal_close_confirmed'] and close['exit_code']==0 and close['child_execution']['returncode']==0 and not close['child_execution']['timeout'] and not close['child_execution']['forced_kill']
 for p,d in m['artifact_sha256'].items():assert sha(p)==d,p
 identity=json.loads((D/'identity.json').read_text());assert identity['complete'] and identity['reset_count']==1 and not identity['internal_reset']
 contacts=json.loads((D/'physics_contacts.json').read_text());assert contacts and all(c['available'] for c in contacts)
 hits=[c['simulation_time_s'] for c in contacts if c['contact']];return min(hits) if hits else 35.
def feature_only_sequence(D):
 D=Path(D);cutoff=verify_sequence(D);history=[];features=[];index=[];inputs={}
 for p in sorted((D/'partial_observations').glob('frame_*.npz')):
  rec=dict(np.load(p));f=int(rec['frame']);t=float(rec['acquisition_time_s'])
  if t>cutoff+1e-8:continue
  imagefile=D/'source_inputs'/p.name;predictionfile=D/'source_predictions'/p.name;images=dict(np.load(imagefile));pred=dict(np.load(predictionfile))
  assert t==float(images['time_s'])==float(pred['time_s']);assert f==int(pred['frame'])
  ids=rec['query_pixel_id'];packet=dict(time_s=t,frame=f,world=rec['world'],query_pixel_id=ids,partner_pixel_id=rec['partner_pixel_id'],sigma_m=rec['sigma_m'],depth_m=pred['depth_m'].ravel()[ids],disparity_native_px=restore_native(pred['disparity_px']).ravel()[ids],left_rgb=images['left_rgb'],right_rgb=images['right_rgb'])
  feature=build_features(packet,history,t);features.append(feature)
  index.extend(dict(frame=f,time_s=t,query_pixel_id=int(q)) for q in ids)
  history.append(dict(frame=f,time_s=t,world=rec['world']));history=[r for r in history if t-r['time_s']<=10.]
  for inp in [p,imagefile,predictionfile]:inputs[str(inp)]=sha(inp)
 return np.vstack(features) if features else np.empty((0,23)),index,inputs

def offline_teacher_labels(D,index):
 """Training/evaluation only, same visible query points; never passed to predictor."""
 D=Path(D);labels=[];height=[];inputs={};cached={}
 for row in index:
  f=row['frame'];p=D/'frames'/f'frame_{f:03d}.npz'
  if f not in cached:cached[f]=dict(np.load(p));inputs[str(p)]=sha(p)
  data=cached[f];assert abs(float(data['time_s'])-row['time_s'])<1e-8
  q=row['query_pixel_id'];y,x=divmod(q,640);depth=float(data['left_depth_m'][y,x]);T=data['left_camera_usd_world'];ray=np.array([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,1.]);point=T[:3,3]+(T[:3,:3]@np.diag([1,-1,-1]))@(ray*depth)
  h=float(point[2]);valid=np.isfinite(depth) and depth>0 and np.isfinite(h)
  label=0 if valid and h<=.05 else 1 if valid and h<=1.5 else -1
  labels.append(label);height.append(h)
 return np.array(labels,np.int8),np.array(height,float),inputs
