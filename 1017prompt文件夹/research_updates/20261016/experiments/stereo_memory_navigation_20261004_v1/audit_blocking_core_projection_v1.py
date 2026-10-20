"""One rejected core's view/metric discrepancy; GT never fed to any actor."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'bank_rock_observation_run_v1';A=H/'persistent_core_blocker_audit_v1/assessment.json';O=H/'blocking_core_projection_audit_v1.json';assert not O.exists()
core=json.loads(A.read_text())['blocking_core_details'];assert len(core)==1;p=np.array(core[0]['xyz']);rows=[]
q=np.load(D/'possible_sets/frame_100.npz');assert core[0]['derived_frames']==[100]
for frame in q['support_frames']:
 frame=int(frame);pose=np.load(D/'estimated_poses'/f'frame_{frame:03d}.npz');gt=np.load(D/'offline_truth'/f'frame_{frame:03d}.npz');pred=np.load(D/'source_predictions'/f'frame_{frame:03d}.npz');mask=np.load(D/'masks'/f'frame_{frame:03d}.npz')
 for eye in ['left','right']:
  T=pose[eye+'_camera_usd_world'];local=(p-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]));uv=local[:2]/local[2]*584.8888888888889+[319.5,239.5];x,y=np.rint(uv).astype(int);inside=local[2]>=.5 and local[2]<=3 and x>=3 and x<636 and y>=3 and y<476
  r=dict(frame=frame,eye=eye,uv=uv.tolist(),core_optical_z=float(local[2]),in_projectable_domain=bool(inside))
  if inside:
   cl=np.load(D/'partial_class_masks'/f'birth_100_{eye}_{frame:03d}.npz')['potential_hazard_mask'];blocked=cv2.dilate(mask[eye].astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool)[y,x];z=float(gt[eye+'_depth_m'][y,x]);r.update(CAD_blocked=bool(blocked),class_positive=bool(cl[y,x]),offline_render_depth_m=z,offline_core_minus_render_axial_m=float(local[2]-z))
   if eye=='left':r.update(current_NN_depth_m=float(pred['depth_m'][y,x]),current_NN_confidence=bool(pred['confidence'][y,x]),core_minus_NN_axial_m=float(local[2]-pred['depth_m'][y,x]))
  rows.append(r)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),core=core[0],rows=rows,only_one_blocking_core=True,no_actor_change_or_refit=True,offline_GT_diagnostic_only=True,whole_research_complete=False,input_sha256=hashlib.sha256(A.read_bytes()).hexdigest(),code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());O.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
