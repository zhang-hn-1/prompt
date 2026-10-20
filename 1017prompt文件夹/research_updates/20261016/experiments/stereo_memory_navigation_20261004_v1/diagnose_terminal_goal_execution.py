"""Posthoc current-state execution audit; no newcontroller/threshold/scoring change."""
from pathlib import Path
import json,numpy as np,hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'terminal_goal_history_run';rows=json.loads((O/'policy_records.json').read_text());ph=np.load(O/'physics.npz');idx=[i for i,r in enumerate(rows) if r['local']['terminal_mode']];i=idx[0];start=5*i+1;t=np.asarray(ph['simulation_time_s']);v=np.linalg.norm(ph['linear_velocity_world_m_s'][:,:2],axis=1);f=lambda x:x.tolist() if isinstance(x,np.ndarray) else x
r=dict(terminal_frame=i,terminal_current_goal_distance_m=rows[i]['local']['terminal_goal_distance_m'],terminal_request_time_s=i*.1,first_applied_terminal_step_index=start,first_applied_terminal_time_s=float(t[start]),goal_end_time_s=float(t[-1]),available_braking_s=float(t[-1]-i*.1),terminal_request_main=float(rows[i]['local']['request'][0]),pre_terminal_main=float(rows[i]['local']['pre_terminal_request'][0]),first_smoothed_main=float(ph['smoothed_target'][start,0]),last_smoothed_main=float(ph['smoothed_target'][-1,0]),first_applied_main=float(ph['applied_action'][start,0]),last_applied_main=float(ph['applied_action'][-1,0]),actual_main_joint_velocity_entry=float(ph['joint_velocity_rad_s'][start,0]),actual_main_joint_velocity_end=float(ph['joint_velocity_rad_s'][-1,0]),XY_speed_entry=float(v[start]),XY_speed_end=float(v[-1]),main_acceleration_limit_rad_s2=.5,minimum_command_ramp_s=float(max(0,abs(ph['smoothed_target'][start-1,0])-.3)/.5),gate_unchanged=True,no_new_parameter_or_sequence=True,whole_research_complete=False)
for field in ['raw_command_target','clipped_target','smoothed_target','applied_action']:
 vals=ph[field][start:,0];hits=np.flatnonzero(abs(vals-.3)<1e-6);r[field+'_reached_terminal_target_time_s']=float(t[start+hits[0]]) if len(hits) else None
(H/'terminal_goal_execution_cause.json').write_text(json.dumps(r,indent=2)+'\n')
fig,ax=plt.subplots(1,2,figsize=(12,4))
for field in ['raw_command_target','smoothed_target','applied_action']:ax[0].plot(t,ph[field][:,0],label=field)
ax[0].plot(t,ph['joint_velocity_rad_s'][:,0],label='measured main joint velocity')
ax[1].plot(t,v,label='measured XY speed')
for a in ax:a.axvline(i*.1,color='black',ls='--',label='terminal event');a.grid(alpha=.2);a.set_xlabel('simulation s');a.legend(fontsize=7)
ax[0].set_ylabel('rad/s');ax[1].set_ylabel('m/s');fig.suptitle('Actual terminal event and execution response: no rescore');fig.tight_layout();fig.savefig(H/'terminal_goal_execution_response.png',dpi=140)
print(json.dumps(r,indent=2))
