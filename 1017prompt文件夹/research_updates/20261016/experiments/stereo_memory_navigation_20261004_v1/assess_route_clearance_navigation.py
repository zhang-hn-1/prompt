"""Actual single-consumer control comparison, causal replay and unchanged gates."""
import json,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from audit_stopping_experiment import assess
from summarize_stage import projection
from route_clearance_policy import InformationPolicy
from online_bank_hold_policy import common_policy_state
from execution_snapshot import ExecutionClone,export_execution
from scripts.action_smoothing import ActionSmoother
from training.action_protocol import load_action_protocol
from roll_pid import BankPID,bank_from_quaternion
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;A=H/'online_bank_hold_navigation_run';B=H/'route_clearance_navigation_run'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=read(H/'route_clearance_navigation_protocol.json');status=read(H/'route_clearance_navigation_status.json')
assert status['status']=='finished' and status['exit_code']==0
assert protocol['frozen_at_utc']<status['start_utc']
for s,v in protocol['source_sha256'].items():assert sha(Path(s))==v,s
a,sa,pa,ra=assess(A);b,sb,pb,rb=assess(B)
events=[i for i,r in enumerate(rb) if r['route_clearance']['entered_this_call']]
assert len(events)==1,events
first=events[0];j=first*5;t=float(pb['simulation_time_s'][j]);n=len(rb)
old_states=read(A/'policy_states.json');new_states=read(B/'policy_states.json')
ea=read(A/'execution_states.json');eb=read(B/'execution_states.json')
for key in pa.files:
 if key not in ('raw_simulation_time_s','raw_physics_step_index'):
  np.testing.assert_array_equal(pa[key][:j+1],pb[key][:j+1],err_msg=key)
for i in range(first):
 assert ra[i]=={k:v for k,v in rb[i].items() if k!='route_clearance'},i
 assert old_states[i]==new_states[i],i
assert ea[:first+1]==eb[:first+1]
for i in range(first+1):
 with np.load(A/'privileged_sensor_packets'/f'frame_{i:03d}.npz') as za,np.load(B/'privileged_sensor_packets'/f'frame_{i:03d}.npz') as zb:
  for key in za.files:np.testing.assert_array_equal(za[key],zb[key])
inputs=sorted((B/'online_inputs').glob('frame_*.npz'))
p=load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml')
def replay(files,last):
 policy=InformationPolicy(p,'visible','points',memory_mode='history');maps=0
 for i in range(last+1):
  with np.load(files[i]) as z:
   with np.load(B/'privileged_sensor_packets'/f'frame_{i:03d}.npz') as packet:
    policy.set_current_packet(dict(points_world=packet['points_world'],root_xyz=packet['root_xyz'],time_s=float(packet['time_s']),speed_xy=rb[i]['speed_xy']))
   action=policy.act(SimpleNamespace(position_world_xy=z['position_world_xy'],yaw_world_rad=float(z['yaw_world_rad'])),z['pointgoal_base_xy'])
  np.testing.assert_array_equal(action,rb[i]['request']);assert policy.records[-1]==rb[i],i
  assert common_policy_state(policy)==new_states[i],i
  snap=B/'belief_snapshots'/f'frame_{i:03d}.npz'
  if snap.exists():
   with np.load(snap) as z:
    for actual,key in [(policy.belief.labels,'labels'),(policy.belief.memory.floor_time,'floor_time'),(policy.belief.memory.hazard_time,'hazard_time'),(policy.belief.provenance(),'provenance')]:np.testing.assert_array_equal(actual,z[key])
   maps+=1
 return policy,maps
event_snapshots=[]
for files in (inputs,inputs[:first+1],inputs[:first+1]+[B/'POISON_FUTURE_MUST_NOT_READ.npz']):
 policy,_=replay(files,first)
 event_snapshots.append(dict(record=policy.records[-1],common=common_policy_state(policy),labels_sha=hashlib.sha256(policy.belief.labels.tobytes()).hexdigest()))
assert event_snapshots[0]==event_snapshots[1]==event_snapshots[2]
_,maps=replay(inputs,n-1)
for key in ('proprio_history','action_history','quaternion_world_wxyz_history'):
 previous=None
 for file in inputs:
  with np.load(file) as z:current=z[key]
  if previous is not None:np.testing.assert_array_equal(previous[1:],current[:-1])
  previous=current.copy()
initial=export_execution(SimpleNamespace(inner=ActionSmoother(.02),pid=BankPID(.4,.1,.7),blocked=False),p)
clone=ExecutionClone.from_snapshot(initial)
for tick in range(len(pb['simulation_time_s'])-1):
 applied,smooth=clone.step(pb['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(pb['root_quaternion_world_wxyz'][tick]),aux_position_rad=pb['joint_position_rad'][tick,1],aux_velocity_rad_s=pb['joint_velocity_rad_s'][tick,1])
 np.testing.assert_array_equal(applied,pb['applied_action'][tick+1]);np.testing.assert_array_equal(smooth,pb['smoothed_target'][tick+1])
 if (tick+1)%5==0:assert clone.snapshot()==eb[(tick+1)//5],tick
requests=np.asarray([r['request'] for r in rb],np.float32)
np.testing.assert_array_equal(pb['raw_command_target'][1:],requests.reshape(-1,2))
common_length=min(len(ra),len(rb));changed_blocks=np.flatnonzero(np.any(np.asarray([r['request'] for r in ra[:common_length]],np.float32)!=requests[:common_length],axis=1))
ticks=min(len(pa['applied_action']),len(pb['applied_action']));changed_ticks=np.flatnonzero(np.any(pa['applied_action'][:ticks]!=pb['applied_action'][:ticks],axis=1))
event_path=np.asarray(old_states[first]['path_xy']);goal=event_path[-1]
end=j+int(round(protocol['local_gate']['window_s']*50))
def local(x,records):
 enough=len(x['simulation_time_s'])>end
 segment=x['position_world_xy'][j:min(end+1,len(x['simulation_time_s']))]
 progress,error=projection(segment,event_path)
 return dict(data_sufficient=bool(enough),path_m=float(np.linalg.norm(np.diff(segment,axis=0),axis=1).sum()),
  original_goal_distance_reduction_m=float(np.linalg.norm(segment[0]-goal)-np.linalg.norm(segment[-1]-goal)),
  event_reference_progress_m=float(progress[-1]-progress[0]),event_reference_lateral_rms_m=float(np.sqrt(np.mean(error**2))),
  event_reference_lateral_peak_m=float(error.max()),
  veto_blocks=int(sum(r['stopping_consumer']['stop_veto'] for r in records[first:min(first+60,len(records))])),
  moving_main_blocks=int(sum(bool(np.asarray(r['request'])[::2].any()) for r in records[first:min(first+60,len(records))])),
  final_original_goal_distance_m=float(np.linalg.norm(segment[-1]-goal)))
la,lb=local(pa,ra),local(pb,rb);tol=protocol['tolerances']['space_m'];meaningful=protocol['local_gate']['meaningful_extra_goal_or_event_reference_progress_m']
local_checks=dict(data_sufficient=lb['data_sufficient'],no_contact=not bool(b['collision']),
 actual_path_nonregression=lb['path_m']>=la['path_m']-tol,
 original_goal_progress_nonregression=lb['original_goal_distance_reduction_m']>=la['original_goal_distance_reduction_m']-tol,
 fewer_veto_blocks=lb['veto_blocks']<la['veto_blocks'],
 meaningful=lb['original_goal_distance_reduction_m']>=la['original_goal_distance_reduction_m']+meaningful or lb['event_reference_progress_m']>=la['event_reference_progress_m']+meaningful)
whole_checks=dict(no_contact=not bool(b['collision']),reference_progress_nonregression=b['independent_reference_progress_final_m']>=a['independent_reference_progress_final_m']-tol,
 lateral_rms_nonregression=b['cross_track_rms_m']<=a['cross_track_rms_m']+tol,lateral_peak_nonregression=b['cross_track_peak_m']<=a['cross_track_peak_m']+tol,
 original_goal_distance_nonregression=b['final_distance_m']<=a['final_distance_m']+tol)
for metrics,records in [(a,ra),(b,rb)]:
 metrics.update(main_motion_blocks=int(sum(bool(np.asarray(r['request'])[::2].any()) for r in records)),veto_blocks=sum(r['stopping_consumer']['stop_veto'] for r in records),
  first_veto_s=next((r['time_s'] for r in records if r['stopping_consumer']['stop_veto']),None),
  no_route_blocks=sum(r['target_base_xy'] is None for r in records),first_no_route_s=next((r['time_s'] for r in records if r['target_base_xy'] is None),None))
oldid=read(A/'experiment_identity.json');newid=read(B/'experiment_identity.json');shared={s:v for s,v in oldid['source_sha256'].items() if s in newid['source_sha256']}
assert all(newid['source_sha256'][s]==v for s,v in shared.items())
result=dict(protocol_before_run=True,actual_runs=1,status=status,baseline=a,candidate=b,baseline_local6s=la,candidate_local6s=lb,
 prefix=dict(planning_event_frame=first,time_s=t,physical_samples=j+1,all17_nonraw_physics_exact=True,
  direct_complete_execution_states_exact=first+1,requests_and_common_states_before_event_exact=first,source_packets_through_event_exact=first+1,
  raw_time_origin_delta_s=float(pb['raw_simulation_time_s'][0]-pa['raw_simulation_time_s'][0]),raw_step_origin_delta=int(pb['raw_physics_step_index'][0]-pa['raw_physics_step_index'][0])),
 interface=dict(first_changed_request_frame=None if not len(changed_blocks) else int(changed_blocks[0]),changed_request_blocks=len(changed_blocks),
  first_changed_applied_tick=None if not len(changed_ticks) else int(changed_ticks[0]),changed_applied_ticks=len(changed_ticks),
  changed_applied_main_ticks=int(np.count_nonzero(pa['applied_action'][:ticks,0]!=pb['applied_action'][:ticks,0])),
  changed_applied_aux_ticks=int(np.count_nonzero(pa['applied_action'][:ticks,1]!=pb['applied_action'][:ticks,1])),
  all_actual_requests_replayed=True,complete_policy_current_past_replay_exact=True,all_maps_replayed_exact=maps,
  measured_feedback_execution_clone_exact=True,history_shifts_once=n-1,future_truncate_poison_event_unchanged=True,shared_source_sha_count=len(shared)),
 event_record=rb[first],local_gate_checks=local_checks,local_gate_passed=all(local_checks.values()),whole_gate_checks=whole_checks,whole_reference_gate_passed=all(whole_checks.values()),
 navigation_accepted=bool(b['success'] and not b['collision']),RGB_estimatedpose_learning_formalbenchmarks_not_admitted=True,
 checkpoint_scope='actual progress along same event-time B0 full reference; no policy index or oldB0 future label for candidate',
 source_sha256={str(f):sha(f) for f in (Path(__file__),H/'route_clearance_navigation_protocol.json',A/'physics.npz',B/'physics.npz')})
(H/'route_clearance_navigation_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
fig,axes=plt.subplots(2,2,figsize=(12,8));ref=np.load(H/'online_provenance_evaluation_reference_e1.npz')['position_world_xy']
axes[0,0].plot(ref[:,0],ref[:,1],':',color='gray',label='frozen evaluation reference')
for label,s,x,records in [('B0 bank hold',sa,pa,ra),('route clearance event',sb,pb,rb)]:
 xy=s['position_world_xy'];progress,_=projection(xy,ref)
 axes[0,0].plot(xy[:,0],xy[:,1],label=label);axes[0,1].plot(s['simulation_time_s'],progress-progress[0],label=label)
 axes[1,0].plot(x['simulation_time_s'],np.linalg.norm(x['linear_velocity_world_m_s'][:,:2],axis=1),label=label)
 axes[1,1].step([r['time_s'] for r in records],[r['request'][0] for r in records],where='post',label=label)
axes[0,0].axis('equal')
for ax,title in zip(axes.flat,['Actual XY m','Independent reference progress m','Actual XY speed m/s','Requested main rad/s']):
 ax.set_title(title);ax.grid(alpha=.2);ax.legend(fontsize=8)
for ax in (axes[0,1],axes[1,0],axes[1,1]):ax.axvline(t,color='black',ls=':');ax.set_xlabel('time s')
fig.tight_layout();fig.savefig(H/'figures/route_clearance_navigation.png',dpi=150);plt.close(fig)
print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','event_record','status')},indent=2))
