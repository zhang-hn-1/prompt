"""Closed-trace execution/physics separation, no new controller or inferred causality."""
from pathlib import Path
import json,numpy as np,datetime
H=Path(__file__).resolve().parent
results={}
for variant in ['B0','velocity']:
 O=H/('velocity_goal_settling_'+variant+'_run_v2');a=json.loads((H/('velocity_goal_settling_'+variant+'_assessment_v2.json')).read_text());p=np.load(O/'physics.npz');e=json.loads((O/'execution_states.json').read_text());start=5*a['nominal_goal_entry_frame']+1;times=p['simulation_time_s'][start:];zero=np.flatnonzero(np.abs(p['smoothed_target'][start:,0])<=1e-8);apzero=np.flatnonzero(np.abs(p['applied_action'][start:,0])<=1e-8)
 assert np.all(p['raw_command_target'][start:]==0)
 speed=np.linalg.norm(p['linear_velocity_world_m_s'][start:,:2],axis=1);v0=p['linear_velocity_world_m_s'][start-1,:2];v0=v0/np.linalg.norm(v0);along=p['linear_velocity_world_m_s'][start:,:2]@v0
 n=a['nominal_goal_entry_frame'];bank=np.array([s['pid']['last']['bank_rad'] for s in e[n:]]);wanted=np.array([s['pid']['last']['desired_bank_rad'] for s in e[n:]]);runs=[];count=0
 for c in a['low_inside_count_trace']:
  count=c['low_inside_count'];runs.append(count)
 result=dict(entry_frame=n,raw_tail_all_zero=True,smoothed_main_zero_delay_s=float(times[zero[0]]-a['nominal_goal_entry_time_s']) if len(zero) else None,applied_main_zero_delay_s=float(times[apzero[0]]-a['nominal_goal_entry_time_s']) if len(apzero) else None,max_measured_speed_m_s=float(speed.max()),final_measured_speed_m_s=float(speed[-1]),speed_peak_after_main_zero_m_s=float(speed[apzero[0]:].max()) if len(apzero) else None,actual_velocity_sign_changes_relative_entry_course=int(np.sum(np.sign(along[1:])!=np.sign(along[:-1]))),bank_entry_rad=float(bank[0]),bank_tail_min_rad=float(bank.min()),bank_tail_max_rad=float(bank.max()),desired_bank_entry_rad=float(wanted[0]),desired_bank_last_rad=float(wanted[-1]),max_distinct_low_inside_samples=max(runs,default=0),no_parameter_or_candidate_change=True,not_proof_of_sole_PID_or_smoother_cause=True)
 results[variant]=result
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),actual_closed_trace_results=results,scope='Zero requests do not imply immediate zero smoothed targets or body speed. Measured rebound identified, no counterfactual attribution, no retune.',old_V1_REJECT_preserved=True)
(H/'velocity_zero_tail_audit_v2.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
