"""Closed actual-response diagnosis, never a model or controller input."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from motion_measurement import xy_curvature
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pid_snapshot(value,frame):
 if value is None and frame==0:return {}
 if not isinstance(value,dict) or not isinstance(value.get('pid'),dict) or not isinstance(value['pid'].get('last'),dict):raise ValueError('only initial preinstallation snapshot may be unavailable')
 return value['pid']['last']
def stats(a):
 a=np.asarray(a,float);a=a[np.isfinite(a)]
 return dict(count=len(a),median=None if not len(a) else float(np.median(a)),p90_abs=None if not len(a) else float(np.percentile(abs(a),90)),mean=None if not len(a) else float(np.mean(a)))
def main():
 output={};inputs={};fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
 phases=json.loads((H/'reverse_main_failure_phase_audit_v1.json').read_text())['phases']
 for col,arm in enumerate(('b0','candidate')):
  D=H/f'reverse_main_navigation_e1_{arm}_run_v1';p=np.load(D/'physics.npz');rows=json.loads((D/'policy_records.json').read_text());t=p['simulation_time_s'];v=p['linear_velocity_world_m_s'][:,:2];xy=p['position_world_xy'];k,course,valid=xy_curvature(t,xy,v);bank=np.array([bank_from_quaternion(q) for q in p['root_quaternion_world_wxyz']]);snap=[pid_snapshot(json.loads((D/'execution'/f'frame_{i:03d}.json').read_text()),i) for i in range(351)];snapshot_time=np.array([float(np.load(D/'frames'/f'frame_{i:03d}.npz')['time_s']) if i%5==0 else rows[i]['time_s'] for i in range(350)]+[float(np.load(D/'frames/frame_350.npz')['time_s'])]);desired=np.array([x.get('desired_bank_rad',np.nan) for x in snap]);error=np.array([x.get('error_rad',np.nan) for x in snap]);blocked=np.array([x.get('downstream_blocked',False) for x in snap]);pidclip=np.array([x.get('pid_clipped',False) for x in snap]);groups=[]
  for ph in phases[arm]:
   a,b=ph['start_frame'],ph['end_frame_exclusive'];start=rows[a]['time_s'];end=rows[b]['time_s'] if b<350 else snapshot_time[-1];mask=(t>start+1e-7)&(t<=end+1e-7);s=np.arange(a+1,b+1);direction=-1 if ph['label']=='normal_reverse' else 1
   groups.append(dict(label=ph['label'],start_frame=a,end_frame_exclusive=b,duration_s=ph['duration_s'],actual_net_goal_axis_m=ph['actual_net_goal_axis_m'],physical_bank_rad=stats(bank[mask]),physical_XY_speed_m_s=stats(np.linalg.norm(v[mask],axis=1)),course_geometric_curvature_1_m=stats(k[mask&valid]),signed_body_convention_curvature_1_m=stats(direction*k[mask&valid]) if ph['label'].startswith('normal_') else None,snapshot_PID_bank_error_rad=stats(error[s]),snapshot_PID_desired_bank_rad=stats(desired[s]),snapshot_downstream_blocked_count=int(blocked[s].sum()),snapshot_PID_clipped_count=int(pidclip[s].sum()),snapshot_count=len(s),actual_curvature_not_target_slope=True))
  output[arm]=groups
  axes[0,col].plot(t,np.rad2deg(bank),label='actual bank (quaternion)');axes[0,col].plot(snapshot_time,np.rad2deg(desired),label='PID target snapshot',alpha=.7);axes[0,col].set_title(arm);axes[0,col].set_ylabel('bank deg');axes[0,col].legend()
  axes[1,col].plot(t,np.linalg.norm(v,axis=1));axes[1,col].set_ylabel('world XY speed m/s')
  axes[2,col].plot(t,k);axes[2,col].set_ylim(-3,3);axes[2,col].set_ylabel('course curvature 1/m');axes[2,col].set_xlabel('simulation time s')
  for ax in axes[:,col]:ax.grid();[ax.axvline(rows[ph['start_frame']]['time_s'],color='gray',alpha=.2) for ph in phases[arm]]
  for file in [D/'physics.npz',D/'policy_records.json',D/'artifact_manifest_post_close.json',*(D/'execution'/f'frame_{i:03d}.json' for i in range(351))]:inputs[str(file)]=sha(file)
 fig.tight_layout();fig.savefig(H/'reverse_main_bank_response_diagnostic_v1.png',dpi=150);plt.close(fig)
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),phases=output,source_sha256={str(Path(__file__).resolve()):sha(__file__),str(H/'motion_measurement.py'):sha(H/'motion_measurement.py')},input_sha256=inputs,measurement_notes=['Quaternion bank uses frozen BankPID definition, not auxiliary joint angle.','PID snapshot last is feedback before last .02s physics update; not exact contemporaneous root state, not future data fed into control.','xy_curvature uses offline centered derivatives and speed>=.05 validity; descriptive only, never online input.','Normal reverse body convention reverses signed course curvature; no model or steady-state generality.','p90 plot clipping is display only; numeric summaries untrimmed.'],causal_fault_proven=False,no_new_solver_physics_fit=True,whole_mission_complete=False)
 out=H/'reverse_main_bank_response_diagnostic_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({a:[dict(label=x['label'],duration=x['duration_s'],bank_error=x['snapshot_PID_bank_error_rad'],speed=x['physical_XY_speed_m_s'],body_k=x['signed_body_convention_curvature_1_m']) for x in g if x['label'].startswith('normal_')] for a,g in output.items()}))
if __name__=='__main__':main()
