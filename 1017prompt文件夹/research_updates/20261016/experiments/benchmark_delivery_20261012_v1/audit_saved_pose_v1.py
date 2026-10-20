from pathlib import Path
import json,numpy as np,hashlib
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
summary=lambda x:dict(rms=float(np.sqrt(np.mean(x*x))),p90=float(np.percentile(x,90)),maximum=float(np.max(x)),final=float(x[-1]))
if __name__=='__main__':
 output=H/'saved_pose_diagnostic_v1.json';assert not output.exists()
 audit=json.loads((H/'existing_evidence_audit_v1.json').read_text());rows=[];inputs={str(H/'existing_evidence_audit_v1.json'):sha(H/'existing_evidence_audit_v1.json'),str(H/'pose_diagnostic_scope_v1.json'):sha(H/'pose_diagnostic_scope_v1.json'),str(Path(__file__)):sha(__file__)}
 for record in audit['matrix']:
  if record['pose_layer']!='estimated_pose':continue
  run=Path(record['run_directory']);p=run/'native_predictions.json';physics=run/'physics.npz';est=json.loads(p.read_text());raw=np.load(physics);time=np.asarray([r['time_s'] for r in est]);np.testing.assert_allclose(time,raw['simulation_time_s'],atol=1e-7,rtol=0);assert len(est)==record['physics_samples'];assert all(r['available'] and r['live_before_next_action'] for r in est)
  xy=np.asarray([r['position_world'][:2] for r in est]);truth=raw['root_position_world_m'][:,:2];np.testing.assert_allclose(xy[0],truth[0],atol=1e-6,rtol=0)
  matrices=np.asarray([r['rotation_world'] for r in est]);axis=matrices[:,:,0];q=raw['root_quaternion_world_wxyz'];actual=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix()[:,:,0];assert np.min(np.linalg.norm(axis[:,:2],axis=1))>1e-4 and np.min(np.linalg.norm(actual[:,:2],axis=1))>1e-4
  yaw=np.arctan2(axis[:,1],axis[:,0]);real=np.arctan2(actual[:,1],actual[:,0]);yawerror=np.abs((yaw-real+np.pi)%(2*np.pi)-np.pi)
  velocity=np.asarray([r['velocity_world_m_s'][:2] for r in est]);verror=np.linalg.norm(velocity-raw['linear_velocity_world_m_s'][:,:2],axis=1)
  rows.append(dict(variant=record['variant'],scene=record['scene'],outcome=record['outcome'],samples=len(est),XY_error_m=summary(np.linalg.norm(xy-truth,axis=1)),heading_error_deg=summary(np.degrees(yawerror)),XY_velocity_error_m_s=summary(verror)))
  inputs[str(p)]=sha(p);inputs[str(physics)]=sha(physics)
 assert len(rows)==6
 output.write_text(json.dumps(dict(rows=rows,inputs_sha256=inputs,initial_gauge_only=True,no_future_online_inputs=True,GT_used_for_diagnostic_only=True,causal_failure_attribution=False,new_physics=0,new_model_queries=0,whole_mission_complete=False),indent=2)+'\n');print(json.dumps(rows))
