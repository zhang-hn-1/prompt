"""Independent fixed-gate assessment; labels never used to generate hold request."""
import json,hashlib,subprocess,sys
from pathlib import Path
import numpy as np
from short_branch import development_schedule
from stop_bank_hold import hold_bank_schedule
from execution_snapshot import ExecutionClone
from guarded_decision_replay import replay_memory
from motion_measurement import qualification
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'))
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent;oldroot=H/'short_branch_runs/development';newroot=H/'stop_bank_hold_run'
protocol=json.loads((H/'stop_bank_hold_protocol.json').read_text());gate=protocol['brake_gate']
a=np.load(oldroot/'physics.npz');b=np.load(newroot/'physics.npz');j=375
for path,sha in protocol['source_sha256'].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,path
identity=json.loads((newroot/'identity.json').read_text())
for path,sha in identity['source_sha256'].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,path
assert identity['actions']==135 and identity['captures']==136 and len(b['simulation_time_s'])==676
requests=np.load(newroot/'requests.npz')['requests'];np.testing.assert_array_equal(requests,hold_bank_schedule(development_schedule()[:75]))
np.testing.assert_array_equal(b['raw_command_target'][1:],requests.reshape(-1,2))
fields=[k for k in a.files if k not in ('raw_physics_step_index','raw_simulation_time_s')]
for k in fields:np.testing.assert_array_equal(a[k][:j+1],b[k][:j+1],err_msg=k)
oldstates=json.loads((oldroot/'execution_states.json').read_text());newstates=json.loads((newroot/'execution_states.json').read_text())
assert oldstates[:76]==newstates[:76]
# Main brake request and its applied/smoothed execution must remain unchanged for entire window.
for k in ('raw_command_target','smoothed_target','applied_action'):np.testing.assert_array_equal(a[k][:,0],b[k][:,0])
changed=int(np.count_nonzero(a['applied_action'][j+1:,1]!=b['applied_action'][j+1:,1]));assert changed>0
clone=ExecutionClone.from_snapshot(newstates[75]);maxerror=0
for tick in range(j,675):
    applied,smoothed=clone.step(b['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(b['root_quaternion_world_wxyz'][tick]),aux_position_rad=b['joint_position_rad'][tick,1],aux_velocity_rad_s=b['joint_velocity_rad_s'][tick,1])
    maxerror=max(maxerror,float(np.max(abs(applied-b['applied_action'][tick+1]))),float(np.max(abs(smoothed-b['smoothed_target'][tick+1]))))
    if (tick+1)%5==0:assert clone.snapshot()==newstates[(tick+1)//5]
assert maxerror==0
hist=sorted((newroot/'history').glob('*.npz'));assert len(hist)==136
for previous,current in zip(hist[:-1],hist[1:]):
    with np.load(previous) as prev,np.load(current) as cur:
        for k in prev.files:np.testing.assert_array_equal(prev[k][1:],cur[k][:-1])
packets=sorted((newroot/'packets').glob('*.npz'));assert len(packets)==136
oldpackets=sorted((oldroot/'packets').glob('*.npz'));prefix_packet_sha_exact=all(hashlib.sha256(x.read_bytes()).hexdigest()==hashlib.sha256(y.read_bytes()).hexdigest() for x,y in zip(oldpackets[:76],packets[:76]))
# Array equality is stronger scientifically than container-byte equality alone.
for f,g in zip(oldpackets[:76],packets[:76]):
    with np.load(f) as x,np.load(g) as y:
        for k in x.files:np.testing.assert_array_equal(x[k],y[k])
def load(f):
    with np.load(f) as x:return {k:x[k] for k in x.files}
maps=[replay_memory(files,75,load) for files in (packets,packets[:76],packets[:76]+[H/'POISON_FUTURE_NOT_READ.npz'])]
for x in maps[1:]:
    np.testing.assert_array_equal(x.labels(x.time),maps[0].labels(maps[0].time));np.testing.assert_array_equal(x.floor_time,maps[0].floor_time);np.testing.assert_array_equal(x.hazard_time,maps[0].hazard_time)
# Schedule only accepts completed prefix: old future replaced/truncated has no effect.
original=np.load(oldroot/'requests.npz')['requests'];poison=original.copy();poison[75:]=np.nan
np.testing.assert_array_equal(hold_bank_schedule(original[:75]),hold_bank_schedule(poison[:75]))
clocks=json.loads((newroot/'camera_clocks.json').read_text());videos={}
for filename in ('third_person.mp4','stereo_view.mp4'):
    v=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-show_entries','stream=nb_read_frames,r_frame_rate,duration','-of','json',str(newroot/filename)]))['streams'][0]
    assert int(v['nb_read_frames'])==136;videos[filename]=v
initialxy=a['position_world_xy'][300];yaw=float(a['yaw_world_rad'][300]);c,s=np.cos(yaw),np.sin(yaw);R=np.array([[c,-s],[s,c]]);goal=initialxy+R@np.array([.8,-.3]);unit=(goal-initialxy)/np.linalg.norm(goal-initialxy)
def metrics(p):
    xy=p['position_world_xy'];time=p['simulation_time_s'];speed=np.linalg.norm(p['linear_velocity_world_m_s'][:,:2],axis=1)
    qualified,counts=qualification(speed[::5]);indices=np.flatnonzero(qualified & (time[::5]>=7.5-1e-5));first=None if not len(indices) else int(indices[0])
    rebound=None if first is None else bool(np.any(speed[::5][first:]>=.05))
    windows=[]
    for horizon in (1.5,3.,6.):
        segment=xy[j:j+round(horizon*50)+1];radial=np.linalg.norm(segment-segment[0],axis=1)
        windows.append(dict(after_zero_s=horizon,path_m=float(np.linalg.norm(np.diff(segment,axis=0),axis=1).sum()),radial_peak_m=float(radial.max()),end_displacement_m=float(radial[-1])))
    bank=np.rad2deg([bank_from_quaternion(q) for q in p['root_quaternion_world_wxyz']]);smoothzero=np.flatnonzero((time>=7.5-1e-5)&(abs(p['smoothed_target'][:,0])<1e-7))
    delta=xy[-1]-initialxy;active=xy[j]-initialxy;stop=xy[-1]-xy[j];lateral=abs(float(delta[0]*unit[1]-delta[1]*unit[0]))
    stop_segment=None if first is None else xy[j:first*5+1]
    return dict(first_qualification_time_s=None if first is None else float(time[::5][first]),qualification_delay_s=None if first is None else float(time[::5][first]-7.5),post_qualification_rebound=rebound,final_low_count=int(counts[-1]),
      qualification_radial_peak_m=None if first is None else float(np.linalg.norm(stop_segment-stop_segment[0],axis=1).max()),qualification_path_m=None if first is None else float(np.linalg.norm(np.diff(stop_segment,axis=0),axis=1).sum()),
      stop_windows=windows,speed_peak_m_s=float(speed[j:].max()),end_one_second_mean_speed_m_s=float(speed[-51:].mean()),final_speed_m_s=float(speed[-1]),bank_at_stop_deg=float(bank[j]),bank_final_deg=float(bank[-1]),bank_peak_after_stop_deg=float(bank[j:].max()),
      smoothed_main_first_zero_s=float(time[smoothzero[0]]),active_delta_initial_yaw_xy_m=(active@R).tolist(),stop_delta_initial_yaw_xy_m=(stop@R).tolist(),whole_delta_initial_yaw_xy_m=(delta@R).tolist(),
      root_yaw_delta_stop_rad=float(p['yaw_world_rad'][-1]-p['yaw_world_rad'][j]),goal_initial_distance_m=float(np.linalg.norm(initialxy-goal)),goal_at_brake_distance_m=float(np.linalg.norm(xy[j]-goal)),goal_final_distance_m=float(np.linalg.norm(xy[-1]-goal)),
      along_frozen_goal_progress_m=float(delta@unit),lateral_to_frozen_goal_ray_m=lateral,scope='localgoal metrics only; stop displacement not counted as accepted navigation')
ma,mb=metrics(a),metrics(b);checks={}
checks['qualified_in_window']=mb['qualification_delay_s'] is not None
checks['qualification_no_later']=checks['qualified_in_window'] and mb['qualification_delay_s']<=gate['qualification_delay_max_s']+gate['time_numeric_tolerance_s']
checks['no_post_qualification_rebound']=mb['post_qualification_rebound'] is False
checks['speed_peak_nonregression']=mb['speed_peak_m_s']<=gate['speed_peak_max_m_s']+gate['speed_numeric_tolerance_m_s']
checks['end_speed_nonregression']=mb['end_one_second_mean_speed_m_s']<=gate['end_one_second_mean_speed_max_m_s']+gate['speed_numeric_tolerance_m_s']
for new,old in zip(mb['stop_windows'],gate['windows']):
    for name in ('path_m','radial_peak_m'):checks[f'{new["after_zero_s"]}s_{name}_nonregression']=new[name]<=old[name]+gate['spatial_numeric_tolerance_m']
checks['meaningful_improvement']=mb['stop_windows'][-1]['radial_peak_m']<=.9*gate['windows'][-1]['radial_peak_m'] or (checks['qualified_in_window'] and mb['qualification_delay_s']<=gate['qualification_delay_max_s']-.2)
accepted=all(checks.values());configuration=json.loads((newroot/'runtime_configuration.json').read_text());contacts=json.loads((newroot/'contacts.json').read_text())
result=dict(protocol_sealed_before_run=protocol['frozen_at_utc']<json.loads((H/'stop_bank_hold_status.json').read_text())['start_utc'],candidate_count=1,
    prefix=dict(all_17_nonraw_physics_exact=True,all_76_complete_snapshots_exact=True,all_75_request_blocks_exact=True,packet_arrays_exact=True,packet_sha_exact=prefix_packet_sha_exact,raw_time_origin_difference_s=float(b['raw_simulation_time_s'][0]-a['raw_simulation_time_s'][0]),raw_step_origin_difference=int(b['raw_physics_step_index'][0]-a['raw_physics_step_index'][0])),
    execution=dict(main_all_raw_smoothed_applied_exact=True,changed_aux_applied_ticks=changed,measured_feedback_reconstruction_ticks=300,measured_feedback_maxerror=0,complete_postbranch_snapshots_exact=True,no_recursive_physical_prediction=True),
    provenance=dict(future_truncation_poison_map_unchanged=True,request_source_future_truncation_poison_unchanged=True,history_shifts_once=135,packets=136,stereo_pairs=len(list((newroot/'frames').glob('*.npz'))),max_camera_clock_age_s=max(abs(d['raw_time_s']-v) for d in clocks for v in d['eyes'].values())),
    counts=dict(request_blocks=135,captures=136,physics=676,videos=videos),runtime_joint_configuration=configuration,obstacle_contacts=sum(bool(d['contact']) for d in contacts),contact_scope='openplane has no obstacles;0 cannot certify safety',
    baseline=ma,candidate=mb,pre_frozen_gate_checks=checks,braking_candidate_accepted=bool(accepted),verdict='ACCEPT bounded brake intervention only' if accepted else 'REJECT held-bank stopping candidate',
    no_navigation_RGB_model_or_formal_scores=True)
assert result['protocol_sealed_before_run']
(H/'stop_bank_hold_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,2,figsize=(12,8))
for p,label in ((a,'B0: return bank to0'),(b,'candidate: hold prior bank22.5deg')):
    time=p['simulation_time_s'][300:]-6;xy=(p['position_world_xy'][300:]-initialxy)@R
    axes[0,0].plot(xy[:,0],xy[:,1],label=label);axes[0,1].plot(time,np.linalg.norm(p['linear_velocity_world_m_s'][300:,:2],axis=1),label=label)
    axes[1,0].plot(time,np.rad2deg([bank_from_quaternion(q) for q in p['root_quaternion_world_wxyz'][300:]]),label=label)
    axes[1,1].plot(time,p['applied_action'][300:,1],label=label)
axes[0,0].scatter([.8],[-.3],marker='*',s=70,c='k');axes[0,0].axis('equal');axes[0,0].set_title('Actual initial-yaw XY (m); localgoal frozen')
axes[0,1].axhline(.05,c='r',ls='--');axes[0,1].set_title('Actual XY speed (m/s)')
axes[1,0].set_title('Measured bank (deg)');axes[1,1].set_title('Applied auxiliary target (rad)')
for ax in (axes[0,1],axes[1,0],axes[1,1]):ax.axvline(1.5,c='k',ls=':');ax.set_xlabel('time from active prefix6s (s)')
for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(H/'figures/stop_bank_hold_validation.png',dpi=150);plt.close(fig)
