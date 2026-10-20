"""Complete right-eye historical label feasibility, without repeating left estimates."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent;c=json.loads((H/'right_action_video_pair_supplement_protocol_v1.json').read_text());L=H/'action_conditioned_video_pair_feasibility_v1';O=H/c['output'];O.mkdir(exist_ok=False);rows=[]
seal=json.loads((L/'artifact_manifest_post_close.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in seal['artifact_sha256'].items())
for name in c['sequences']:
 D=H/name;F=L/(name+'_inputs.npz');features=np.load(F);xy=features['query_xy'].astype(np.float32).reshape(-1,1,2);indices=features['current_capture_indices'];inputhash=hashlib.sha256(F.read_bytes()).hexdigest();(O/(name+'_input_reference_before_future_labels.json')).write_text(json.dumps(dict(current_causal_inputs=str(F),sha256=inputhash,no_input_or_split_change=True),indent=2)+'\n')
 cap=cv2.VideoCapture(str(D/'stereo_view.mp4'));images=[]
 while True:
  ok,img=cap.read()
  if not ok:break
  assert img.shape[:2]==(480,1280);images.append(cv2.cvtColor(img[:,640:],cv2.COLOR_BGR2GRAY))
 cap.release();assert len(images)==len(indices)+2;targets=[];valids=[]
 for i in indices:
  i=int(i);forward,ok,error=cv2.calcOpticalFlowPyrLK(images[i],images[i+1],xy,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01));back,bok,_=cv2.calcOpticalFlowPyrLK(images[i+1],images[i],forward,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01));valid=ok.ravel().astype(bool)&bok.ravel().astype(bool)&np.isfinite(forward[:,0]).all(1)&(np.linalg.norm(back[:,0]-xy[:,0],axis=1)<=1)&(error.ravel()<=20);targets.append(forward[:,0]-xy[:,0]);valids.append(valid)
 np.savez_compressed(O/(name+'_future_right_flow_labels.npz'),flow=np.asarray(targets),usable_LK=np.asarray(valids),approximate_video_teacher=True,not_3D_or_free_truth=True)
 assert hashlib.sha256(F.read_bytes()).hexdigest()==inputhash;row=dict(sequence=name,pairs=len(indices),usable_right_labels=int(np.asarray(valids).sum()),causal_input_reference=str(F),input_sha256=inputhash,historical_not_fresh=True,not_stereo_depth_certification=True);rows.append(row);print(row,flush=True)
(O/'assessment.json').write_text(json.dumps(dict(protocol=c,rows=rows,no_model_fit=True,no_actor_change=True,no_next_frame_model_predictions_yet=True,not_worldmodel_or_navigation_admission=True),indent=2)+'\n');(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.rglob('*') if p.is_file()}),indent=2)+'\n')
