"""Post-run scoring only. Renderer labels never reach policy or stereo inference."""
from pathlib import Path
import json,numpy as np,cv2

def score(out):
 out=Path(out);term=json.loads((out/'termination.json').read_text());states=json.loads((out/'states.json').read_text());contacts=json.loads((out/'physics_contacts.json').read_text())
 ep=json.loads((out/'task_episode.json').read_text())['actual_episode'];goal=np.array(ep['goal_position_world_m'][:2]);dist=[float(np.linalg.norm(np.array(s['position'][:2])-goal)) for s in states]
 errors=[];near_valid=0;source_points=0;paired_count=0
 for p in sorted((out/'source_predictions').glob('*.npz')):
  pred=np.load(p);truth=np.load(out/'frames'/p.name);z=pred['depth_m'];gt=truth['left_depth_m'];gt=np.asarray(gt).squeeze()
  assert gt.shape==z.shape,(gt.shape,z.shape)
  keep=pred['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(gt)&(gt>0)
  errors.append(np.abs(z[keep]-gt[keep]));near_valid+=int((keep&(z<1)).sum())
  packet=np.load(out/'sensor_packets'/p.name.replace('frame_','source_'));source_points+=len(packet['world']);paired_count+=1
 e=np.concatenate(errors) if errors else np.empty(0);depth_pass=len(e)>=1000 and float(np.quantile(e,.9))<=.15 and float((e>.3).mean())<=.1
 # Preserve actual collision record schema; reject unavailable contact samples.
 collided=[];unavailable=0
 for c in contacts:
  if c['contact']:collided.append(c)
  if not c['available']:unavailable+=1
 video={}
 for p in out.glob('*.mp4'):
  cap=cv2.VideoCapture(str(p));n=0
  while True:
   ok,frame=cap.read()
   if not ok:break
   n+=1
  cap.release();video[p.name]=n
 result=dict(termination=term,initial_distance_m=dist[0],remaining_m=dist[-1],net_progress_m=dist[0]-dist[-1],captures=len(states),contact_schema_example=contacts[0] if contacts else None,contact_records=len(contacts),contacts_detected_by_named_fields=len(collided),unavailable_contacts=unavailable,depth_scoring=dict(pixels=len(e),p90_absolute_error_m=float(np.quantile(e,.9)) if len(e) else None,outlier_fraction_above_0p3m=float((e>.3).mean()) if len(e) else None,under1m_confident_pixels=near_valid,source_points=source_points,saved_pairs=paired_count,pass_gate=bool(depth_pass),scope='all confident range0.5-3m pixels, includes robot surfaces; conditional measurement audit, not full coverage'),video_frames=video)
 result['raw_arrival_pass']=bool(dist[-1]<=.2 and 0<=float(states[-1]['time_s'])<=90.+1e-5 and abs(float(states[-1]['time_s'])-float(term['time_s']))<=1e-5)
 result['first_contact_s']=min((float(c['simulation_time_s']) for c in collided),default=None)
 result['retain']=bool(term['termination']=='success' and result['raw_arrival_pass'] and depth_pass and unavailable==0 and len(collided)==0 and len(contacts)>0 and set(video)=={'stereo_view.mp4','third_person.mp4'} and all(n==len(states) for n in video.values()))
 return result
if __name__=='__main__':
 import sys
 r=score(sys.argv[1]);print(json.dumps(r,indent=2))
