"""Post-close measurement comparison only, never actor inputs or future predictors."""
from pathlib import Path
import json,numpy as np,hashlib
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
out={}
for arm in ('b0','paced'):
 D=H/('rolling_endpoint_native_e0_'+arm+'_run_v1');status=json.loads((H/('rolling_endpoint_native_e0_'+arm+'_status_v1.json')).read_text());assert status['status']=='finished' and status['child_returncode']==0
 seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
 for p,d in seal['artifact_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==d,p
 states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());episode=json.loads((D/'task_episode.json').read_text())['actual_episode'];goal=np.asarray(episode['goal_position_world_m'][:2]);start=np.asarray(episode['start_position_world_m'][:2]);axis=(goal-start)/np.linalg.norm(goal-start)
 samples=[];phases={};reasons={}
 for r in rows:
  f=r['frame'];s=states[f];n=r['native_current_pose'];p=np.asarray(n['root_position_world_m']);v=np.asarray(n['velocity_world_m_s']);truep=np.asarray(s['position']);truev=np.asarray(s['velocity']);R=np.asarray(n['root_rotation_world']);truthR=Rotation.from_quat(np.asarray(s['quaternion'])[[1,2,3,0]]).as_matrix()
  samples.append(dict(frame=f,time_s=r['time_s'],XY_pose_error_m=float(np.linalg.norm((p-truep)[:2])),native_speed_m_s=float(np.linalg.norm(v[:2])),actual_speed_m_s=float(np.linalg.norm(truev[:2])),XY_velocity_vector_error_m_s=float(np.linalg.norm((v-truev)[:2])),rotation_error_degree=float(Rotation.from_matrix(R.T@truthR).magnitude()*180/np.pi),native_goal_remaining_m=float(np.linalg.norm(p[:2]-goal)),actual_goal_remaining_m=float(np.linalg.norm(truep[:2]-goal)),main_request=float(r['request'][0]),reference_version=r['local_reference_version']))
  k=str(r['local_reference_version']);phase=phases.setdefault(k,dict(frames=[],actual_axis_gain_m=0.,path_length_m=0.,changes=0,reserve_restores=0))
  phase['frames'].append(f);delta=np.asarray(states[f+1]['position'][:2])-truep[:2];phase['actual_axis_gain_m']+=float(delta@axis);phase['path_length_m']+=float(np.linalg.norm(delta));phase['changes']+=int(r.get('rolling_control_applied',False));phase['reserve_restores']+=int('preserve_original_within_same_reserve' in r.get('measured_speed_control_reasons',[]))
  for reason in r.get('measured_speed_control_reasons',[]):reasons[reason]=reasons.get(reason,0)+1
 for phase in phases.values():phase['time_s']=len(phase['frames'])*.1;phase['first_frame']=phase['frames'][0];phase['last_frame']=phase['frames'][-1];del phase['frames']
 errors=np.array([s['XY_pose_error_m'] for s in samples]);sv=np.array([s['native_speed_m_s']-s['actual_speed_m_s'] for s in samples]);ve=np.array([s['XY_velocity_vector_error_m_s'] for s in samples]);rot=np.array([s['rotation_error_degree'] for s in samples]);last=samples[-1]
 out[arm]=dict(current_pose_XY_RMS_m=float(np.sqrt(np.mean(errors**2))),current_pose_XY_peak_m=float(max(errors)),last_before_action=last,native_minus_actual_speed_mean_m_s=float(np.mean(sv)),speed_MAE_m_s=float(np.mean(abs(sv))),velocity_vector_RMS_m_s=float(np.sqrt(np.mean(ve**2))),rotation_RMS_degree=float(np.sqrt(np.mean(rot**2))),phases=phases,reasons=reasons,samples=samples)
r=dict(arms=out,post_close_truth_comparison_only=True,GT_not_added_to_runtime_inputs=True,not_unique_causal_attribution=True,no_action_or_gate_changes=True,whole_mission_complete=False)
p=H/'rolling_endpoint_native_layer_audit_v1.json';assert not p.exists();p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:{n:v for n,v in d.items() if n not in ('samples','phases')} for k,d in out.items()}))
