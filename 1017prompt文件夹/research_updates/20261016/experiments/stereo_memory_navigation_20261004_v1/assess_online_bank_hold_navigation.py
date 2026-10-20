"""Actual online consumer, causal inputs, frozen stop and independent path gates."""
import json,hashlib,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from audit_stopping_experiment import assess
from summarize_stage import projection
from online_bank_hold import OnlineBankHold
from online_bank_hold_policy import InformationPolicy,common_policy_state
from stopping_policy import InformationPolicy as ParentPolicy
from execution_snapshot import export_execution,ExecutionClone
from motion_measurement import qualification
from scripts.action_smoothing import ActionSmoother
from training.action_protocol import load_action_protocol
from roll_pid import BankPID,bank_from_quaternion
H=Path(__file__).resolve().parent;oldroot=H/'local_stopping_run';root=H/'online_bank_hold_navigation_run';read=lambda p:json.loads(p.read_text())
protocol=read(H/'online_bank_hold_navigation_protocol.json');limits=protocol['numeric_tolerances']
for f,sha in protocol['source_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha,f
old,oldstate,pa,ra=assess(oldroot);new,newstate,pb,rb=assess(root)
entered=[i for i,r in enumerate(rb) if r['bank_stop_hold']['entered']];assert entered
first=entered[0];j=first*5;t=float(pb['simulation_time_s'][j]);count=len(rb)
assert first==protocol['first_stop_baseline']['event_index'],'same-prefix control comparison gate failed'
for key in pa.files:
 if key not in ('raw_simulation_time_s','raw_physics_step_index'):np.testing.assert_array_equal(pa[key][:j+1],pb[key][:j+1],err_msg=key)
for i in range(first+1):
 comparison={k:v for k,v in rb[i].items() if k not in ('bank_stop_hold','bank_stop_state_before','baseline_final_request')};comparison['request']=rb[i]['baseline_final_request']
 assert comparison==ra[i],i
for i in range(first+1):
 with np.load(oldroot/'privileged_sensor_packets'/f'frame_{i:03d}.npz') as a,np.load(root/'privileged_sensor_packets'/f'frame_{i:03d}.npz') as b:
  for k in a.files:np.testing.assert_array_equal(a[k],b[k])
# Clone deterministic execution under measured CURRENT feedback, not physical recursion.
p=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
initial=export_execution(SimpleNamespace(inner=ActionSmoother(.02),pid=BankPID(.4,.1,.7),blocked=False),p)
snapshots=read(root/'execution_states.json');assert snapshots[0] is None and len(snapshots)==count+1
for name,x in [('baseline',pa),('candidate',pb)]:
 clone=ExecutionClone.from_snapshot(initial);maxerr=0
 end=j if name=='baseline' else len(x['simulation_time_s'])-1
 for tick in range(end):
  applied,smooth=clone.step(x['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(x['root_quaternion_world_wxyz'][tick]),aux_position_rad=x['joint_position_rad'][tick,1],aux_velocity_rad_s=x['joint_velocity_rad_s'][tick,1])
  maxerr=max(maxerr,float(np.max(abs(applied-x['applied_action'][tick+1]))),float(np.max(abs(smooth-x['smoothed_target'][tick+1]))))
  if (tick+1)%5==0:assert clone.snapshot()==snapshots[(tick+1)//5],(name,tick)
 assert maxerr==0
# Reproduce real latch output from current parent request, never old future B0 actions.
latch=OnlineBankHold()
for i,r in enumerate(rb):
 assert latch.snapshot()==r['bank_stop_state_before']
 final,info=latch.apply(r['baseline_final_request']);np.testing.assert_array_equal(final,r['request']);assert info==r['bank_stop_hold']
 np.testing.assert_array_equal(np.asarray(r['baseline_final_request'])[0::2],np.asarray(r['request'])[0::2])
requests=np.asarray([r['request'] for r in rb],np.float32)
np.testing.assert_array_equal(pb['raw_command_target'][1:],requests.reshape(-1,2))
main_series_equal={k:bool(np.array_equal(pa[k][:min(len(pa[k]),len(pb[k])),0],pb[k][:min(len(pa[k]),len(pb[k])),0])) for k in ('raw_command_target','smoothed_target','applied_action')}
changed_aux=int(np.count_nonzero(pa['applied_action'][j+1:len(pb['applied_action']),1]!=pb['applied_action'][j+1:,1]));assert changed_aux>0
# Complete parent-policy/map reconstruction on prefix. New actual input files only.
inputs=sorted((root/'online_inputs').glob('*.npz'));states=read(root/'policy_states.json')
def load_input(f):
 with np.load(f) as z:return {k:z[k] for k in z.files}
def replay(files,last,policyclass):
 policy=policyclass(p,'visible','points',memory_mode='history');checks=0
 for i in range(last+1):
  z=load_input(files[i]);packet=np.load(root/'privileged_sensor_packets'/f'frame_{i:03d}.npz')
  policy.set_current_packet(dict(points_world=packet['points_world'],root_xyz=packet['root_xyz'],time_s=float(packet['time_s']),speed_xy=rb[i]['speed_xy']))
  obs=SimpleNamespace(position_world_xy=z['position_world_xy'],yaw_world_rad=float(z['yaw_world_rad']))
  action=policy.act(obs,z['pointgoal_base_xy']);expected=rb[i]['request'] if policyclass is InformationPolicy else rb[i]['baseline_final_request']
  np.testing.assert_array_equal(action,expected);assert common_policy_state(policy)==states[i]
  if policyclass is InformationPolicy:assert policy.records[-1]==rb[i]
  snapshot=root/'belief_snapshots'/f'frame_{i:03d}.npz'
  if snapshot.exists():
   with np.load(snapshot) as snap:
    for a,b in [(policy.belief.labels,snap['labels']),(policy.belief.memory.floor_time,snap['floor_time']),(policy.belief.memory.hazard_time,snap['hazard_time']),(policy.belief.provenance(),snap['provenance'])]:np.testing.assert_array_equal(a,b)
   checks+=1
 return policy,checks
parent,_=replay(inputs,first,ParentPolicy)
causal=[]
for files in (inputs,inputs[:first+1],inputs[:first+1]+[root/'POISON_FUTURE_NOT_READ.npz']):
 policy,_=replay(files,first,InformationPolicy)
 assert policy.bank_stop.snapshot()==rb[first]['bank_stop_hold']['state'];np.testing.assert_array_equal(policy.belief.labels,parent.belief.labels)
 causal.append(True)
full,snapshotchecks=replay(inputs,count-1,InformationPolicy)
for key in ('proprio_history','action_history','quaternion_world_wxyz_history'):
 previous=None
 for file in inputs:
  with np.load(file) as z:cur=z[key]
  if previous is not None:np.testing.assert_array_equal(previous[1:],cur[:-1])
  previous=cur.copy()
# Score real physical stop, independent of action being nonzero in auxiliary channel.
def stop_metrics(x):
 times=x['simulation_time_s'];xy=x['position_world_xy'];speed=np.linalg.norm(x['linear_velocity_world_m_s'][:,:2],axis=1)
 resume=np.flatnonzero((times>t+1e-5)&(x['raw_command_target'][:,0]!=0));end_interval=int(resume[0]) if len(resume) else len(times)
 qq,counts=qualification(speed[::5]);qindex=np.flatnonzero(qq & (times[::5]>=t-1e-5) & (np.arange(len(qq))*5<end_interval));qfirst=None if not len(qindex) else int(qindex[0])*5
 comparable=times[-1]>=t+6-1e-5 and end_interval>j+300
 windows=[]
 for horizon in (1.5,3.,6.):
  end=j+round(horizon*50)
  if end>=len(times):windows.append(dict(horizon_s=horizon,data_sufficient=False));continue
  segment=xy[j:end+1];windows.append(dict(horizon_s=horizon,data_sufficient=True,path_m=float(np.linalg.norm(np.diff(segment,axis=0),axis=1).sum()),radial_peak_m=float(np.linalg.norm(segment-segment[0],axis=1).max())))
 segment=xy[j:];bank=np.rad2deg([bank_from_quaternion(q) for q in x['root_quaternion_world_wxyz']]);zero=np.flatnonzero((times>t+1e-5)&(abs(x['smoothed_target'][:,0])<1e-7));h=min(j+300,len(times)-1)
 return dict(event_time_s=t,qualification_time_s=None if qfirst is None else float(times[qfirst]),qualification_delay_s=None if qfirst is None else float(times[qfirst]-t),
  post_qualification_rebound=None if qfirst is None else bool(np.any(speed[::5][qfirst//5:(end_interval+4)//5]>=.05)),
  first_resume_time_s=None if not len(resume) else float(times[resume[0]]),first6s_comparable=comparable,windows=windows,speed_peak_first6s_m_s=float(speed[j:h+1].max()),end_one_second_mean_speed_first6s_m_s=None if not comparable else float(speed[j+250:j+301].mean()),
  actual_bank_at_event_deg=float(bank[j]),actual_bank_final_deg=float(bank[-1]),first_smoothed_main_zero_s=None if not len(zero) else float(times[zero[0]]),
  entire_postevent_path_m=float(np.linalg.norm(np.diff(segment,axis=0),axis=1).sum()),entire_postevent_radial_peak_m=float(np.linalg.norm(segment-segment[0],axis=1).max()),actual_final_xy_speed_m_s=float(speed[-1]))
sa,sb=stop_metrics(pa),stop_metrics(pb);base=protocol['first_stop_baseline'];brake={}
brake['comparable']=sb['first6s_comparable'];brake['qualification_no_later']=sb['qualification_delay_s'] is not None and sb['qualification_delay_s']<=base['qualification_delay_s']+limits['time_s'];brake['no_rebound']=sb['post_qualification_rebound'] is False
brake['speed_peak_nonregression']=sb['speed_peak_first6s_m_s']<=base['speed_peak_m_s']+limits['speed_m_s']
brake['end_mean_speed_nonregression']=sb['end_one_second_mean_speed_first6s_m_s'] is not None and sb['end_one_second_mean_speed_first6s_m_s']<=base['end_one_second_mean_speed_m_s']+limits['speed_m_s']
for b,a in zip(sb['windows'],base['windows']):
 for key in ('path_m','radial_peak_m'):brake[f'{a["horizon_s"]}s_{key}_nonregression']=b['data_sufficient'] and b[key]<=a[key]+limits['space_m']
brake['meaningful_improvement']=(sb['windows'][-1]['data_sufficient'] and sb['windows'][-1]['radial_peak_m']<=.9*base['windows'][-1]['radial_peak_m']) or (sb['qualification_delay_s'] is not None and sb['qualification_delay_s']<=base['qualification_delay_s']-.2)
g=protocol['whole_reference_gate'];whole=dict(progress_nonregression=new['independent_reference_progress_final_m']>=g['min_final_progress_m']-limits['space_m'],lateral_rms_nonregression=new['cross_track_rms_m']<=g['max_lateral_rms_m']+limits['space_m'],lateral_peak_nonregression=new['cross_track_peak_m']<=g['max_lateral_peak_m']+limits['space_m'],final_goal_nonregression=new['final_distance_m']<=g['max_final_goal_distance_m']+limits['space_m'],no_contact=not bool(new['collision']),meaningful=new['independent_reference_progress_final_m']>=g['min_final_progress_m']+.05 or new['cross_track_rms_m']<=.9*g['max_lateral_rms_m'])
# Explicit rejection reason: braking does not resume or complete a blocked path.
new.update(main_motion_blocks=int(np.count_nonzero(np.any(requests[:,0::2]!=0,axis=1))),bank_hold_blocks=sum(r['bank_stop_hold']['active'] for r in rb),hold_entry_times_s=[rb[i]['time_s'] for i in entered],last_route_reason=rb[-1]['route_reason'])
oldid=read(oldroot/'experiment_identity.json');newid=read(root/'experiment_identity.json');shared={f:sha for f,sha in oldid['source_sha256'].items() if f in newid['source_sha256']};assert all(newid['source_sha256'][f]==sha for f,sha in shared.items())
result=dict(role='one actual35s same-start E1 online stop-bank-source diagnostic; privileged depth+truepose',protocol_before_run=protocol['frozen_at_utc']<read(H/'online_bank_hold_navigation_status.json')['start_utc'],
 prefix=dict(first_event_frame=first,time_s=t,physics_samples=j+1,all17nonraw_fields_exact=True,all_first_event_parent_records_exact=True,source_packet_arrays_exact=True,baseline_execution_reconstructed_equals_live_snapshot=True,common_policy_states_replayed_exact=True,raw_time_origin_delta_s=float(pb['raw_simulation_time_s'][0]-pa['raw_simulation_time_s'][0]),raw_step_origin_delta=int(pb['raw_physics_step_index'][0]-pa['raw_physics_step_index'][0])),
 interface=dict(parent_called_once_per_capture=True,all_latch_requests_replayed_exact=True,all_actual_requests_match=True,all_main_raw_smoothed_applied_exact=all(main_series_equal.values()),main_series_equal=main_series_equal,changed_aux_applied_ticks=changed_aux,whole_candidate_execution_measured_feedback_exact=True,future_input_truncate_poison_unchanged=all(causal),all_saved_memory_snapshots_replayed=snapshotchecks,common_states_all_exact=True,history_shifts_once=count-1,actual_complete_execution_snapshots=count+1,shared_frozen_source_count=len(shared)),
 baseline=old,candidate=new,baseline_stop=sa,candidate_stop=sb,brake_gate_checks=brake,brake_gate_passed=all(brake.values()),whole_reference_gate_checks=whole,whole_reference_gate_passed=all(whole.values()),navigation_accepted=bool(new['success'] and not new['collision']),
 old_stopping_bound_still_uncertified=True,RGB_and_estimatedpose_learning_formalbenchmarks_not_admitted=True,conclusion='judge independent brake/interface/reference/navigation layers; parking is never navigation acceptance')
assert result['protocol_before_run'];(H/'online_bank_hold_navigation_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ref=np.load(H/'online_provenance_evaluation_reference_e1.npz')['position_world_xy'];fig,axes=plt.subplots(2,2,figsize=(12,8));axes[0,0].plot(ref[:,0],ref[:,1],':',c='gray',label='frozen independent ref')
for label,s,x in [('B0 stop and return bank',oldstate,pa),('online bank-source hold',newstate,pb)]:
 xy=s['position_world_xy'];prog,lat=projection(xy,ref);axes[0,0].plot(xy[:,0],xy[:,1],label=label);axes[0,1].plot(s['simulation_time_s'],prog-prog[0],label=label)
 axes[1,0].plot(x['simulation_time_s'],np.linalg.norm(x['linear_velocity_world_m_s'][:,:2],axis=1),label=label);axes[1,1].plot(x['simulation_time_s'],np.rad2deg([bank_from_quaternion(q) for q in x['root_quaternion_world_wxyz']]),label=label)
axes[0,0].axis('equal');axes[0,0].set_title('Actual XY (m)');axes[0,1].set_title('Independent reference progress (m)');axes[1,0].axhline(.05,c='r',ls='--');axes[1,0].set_title('Actual XY speed (m/s)');axes[1,1].set_title('Measured bank (deg)')
for ax in (axes[0,1],axes[1,0],axes[1,1]):ax.axvline(t,c='k',ls=':');ax.set_xlabel('time (s)')
for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(H/'figures/online_bank_hold_navigation.png',dpi=150);plt.close(fig)
