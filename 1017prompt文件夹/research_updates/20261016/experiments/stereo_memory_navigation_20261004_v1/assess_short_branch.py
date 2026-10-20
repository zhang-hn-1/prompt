"""Independent short whole-plan, stop-branch, repeat-applicability audit."""
import hashlib,json,subprocess
from pathlib import Path
import numpy as np
from short_branch import development_schedule,ShortConsumer
from guarded_decision_replay import replay_memory
from guarded_response import physical_state
from execution_snapshot import ExecutionClone
from motion_measurement import qualification
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent;roots={k:H/'short_branch_runs'/k for k in ('development','fresh')};p={k:np.load(r/'physics.npz') for k,r in roots.items()};protocol=json.loads((H/'short_branch_protocol.json').read_text());out={}
for phase,r in roots.items():
    x=p[phase];states=json.loads((r/'execution_states.json').read_text());clocks=json.loads((r/'camera_clocks.json').read_text());requests=np.load(r/'requests.npz')['requests'];blocks=135 if phase=='development' else 210
    assert len(requests)==blocks and len(x['simulation_time_s'])==blocks*5+1 and len(states)==blocks+1
    np.testing.assert_array_equal(x['raw_command_target'][1:],requests.reshape(-1,2));assert not requests[:60].any()
    if phase=='development':np.testing.assert_array_equal(requests,development_schedule())
    clone=ExecutionClone.from_snapshot(states[60]);err=0.;snapmatch=True
    for tick in range(300,blocks*5):
        a,s=clone.step(x['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(x['root_quaternion_world_wxyz'][tick]),aux_position_rad=x['joint_position_rad'][tick,1],aux_velocity_rad_s=x['joint_velocity_rad_s'][tick,1])
        err=max(err,float(np.max(abs(a-x['applied_action'][tick+1]))),float(np.max(abs(s-x['smoothed_target'][tick+1]))))
        if (tick+1)%5==0:snapmatch&=clone.snapshot()==states[(tick+1)//5]
    assert err==0 and snapmatch
    hist=sorted((r/'history').glob('*.npz'))
    for a,b in zip(hist[:-1],hist[1:]):
        with np.load(a) as prev,np.load(b) as cur:
            for key in prev.files:np.testing.assert_array_equal(prev[key][1:],cur[key][:-1])
    for file,h in json.loads((r/'identity.json').read_text())['source_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==h
    videos={}
    for f in ('third_person.mp4','stereo_view.mp4'):
        v=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-show_entries','stream=nb_read_frames,r_frame_rate,duration','-of','json',str(r/f)]))['streams'][0]
        assert int(v['nb_read_frames'])==blocks+1;videos[f]=v
    speed=np.linalg.norm(x['linear_velocity_world_m_s'][::5,:2],axis=1);q,count=qualification(speed);after=np.flatnonzero(q & (x['simulation_time_s'][::5]>=7.5-1e-5));stop=None if not len(after) else float(x['simulation_time_s'][::5][after[0]])
    j=375;xy=x['position_world_xy'];bank=np.array([bank_from_quaternion(v) for v in x['root_quaternion_world_wxyz']]);stoprows=[]
    for horizon in (1.5,3.,6.):
        end=j+round(horizon*50);segment=xy[j:end+1];radial=np.linalg.norm(segment-segment[0],axis=1)
        stoprows.append(dict(after_zero_s=horizon,path_m=float(np.linalg.norm(np.diff(segment,axis=0),axis=1).sum()),radial_peak_m=float(radial.max()),end_displacement_m=float(radial[-1])))
    if stop is not None:
        end=round(stop*50);seg=xy[j:end+1];stop_at=dict(time_s=stop,delay_from_zero_s=stop-7.5,path_m=float(np.linalg.norm(np.diff(seg,axis=0),axis=1).sum()),radial_peak_m=float(np.linalg.norm(seg-seg[0],axis=1).max()))
    else:stop_at=None
    smootherzero=np.flatnonzero((x['simulation_time_s']>=7.5-1e-5)&(abs(x['smoothed_target'][:,0])<1e-7))
    initialyaw=float(x['yaw_world_rad'][300]);c,s=np.cos(initialyaw),np.sin(initialyaw);R=np.array([[c,s],[-s,c]])
    active_delta=(xy[375]-xy[300])@R.T;whole_delta=(xy[675]-xy[300])@R.T
    out[phase]=dict(counts=dict(requests=blocks,captures=blocks+1,physics=blocks*5+1,history_shifts_once=blocks,stereo_pairs=len(list((r/'frames').glob('*.npz'))),packets=len(list((r/'packets').glob('*.npz')))),videos=videos,
        execution_measured_feedback_exact=True,complete_snapshots_exact=True,camera_age_max_s=max(abs(row['raw_time_s']-t) for row in clocks for t in row['eyes'].values()),
        active_delta_local_xy_m=active_delta.tolist(),active_yaw_delta_rad=float(x['yaw_world_rad'][375]-x['yaw_world_rad'][300]),whole_short_delta_local_xy_m=whole_delta.tolist(),
        speed_at_zero_m_s=float(np.linalg.norm(x['linear_velocity_world_m_s'][375,:2])),bank_at_zero_deg=float(np.rad2deg(bank[375])),smoothed_main_first_zero_s=float(x['simulation_time_s'][smootherzero[0]]),
        first_stop_qualification=stop_at,stop_windows=stoprows,speed_peak_after_zero_until_6s_m_s=float(np.linalg.norm(x['linear_velocity_world_m_s'][375:676,:2],axis=1).max()))
# Current/past-only decision and observed provenance exact under real tail poison.
entries=json.loads((H/'short_response_library.json').read_text())
for e in entries:
    for k in ('xy','times','requests'):e[k]=np.asarray(e[k])
consumer=ShortConsumer(entries);fresh=roots['fresh'];decisions=json.loads((fresh/'decisions.json').read_text());packetfiles=sorted((fresh/'packets').glob('*.npz'));checks=[]
def load(f):
    with np.load(f) as z:return {k:z[k] for k in z.files}
def convert(v):
    if isinstance(v,np.ndarray):return v.tolist()
    raise TypeError(type(v))
for d in decisions:
    frame=d['frame'];answers=[]
    for files in (packetfiles,packetfiles[:frame+1],packetfiles[:frame+1]+[fresh/'POISON_FUTURE_DO_NOT_READ.npz']):
        memory=replay_memory(files,frame,load)
        result=consumer.select(d['current'],d['execution'],d['goal_world'],memory.labels(memory.time),memory.origin,memory.resolution)
        answers.append(result);assert json.dumps(result,default=convert,sort_keys=True)==json.dumps(d['result'],sort_keys=True)
    mf=np.load(fresh/f'decision_map_{frame:03d}.npz');np.testing.assert_array_equal(memory.labels(memory.time),mf['labels']);np.testing.assert_array_equal(memory.floor_time,mf['floor_time']);np.testing.assert_array_equal(memory.hazard_time,mf['hazard_time'])
    assert json.loads((fresh/'execution_states.json').read_text())[frame]==d['execution']
    checks.append(dict(frame=frame,selected=d['result']['selected'],status=d['result']['status'],future_tail_truncate_poison_unchanged=True,causal_memory_and_current_snapshot_exact=True,checked=d['result']['checked']))
first=decisions[0];assert first['result']['selected']=='short_positive';req=np.load(fresh/'requests.npz')['requests'];np.testing.assert_array_equal(req[60:135],first['result']['requests'])
old=p['development'];new=p['fresh'];pred=np.asarray(first['result']['world_xy']);actual=new['position_world_xy'][300:676];xyerror=np.linalg.norm(pred-actual,axis=1)
oldyaw=np.unwrap(old['yaw_world_rad'][300:676]);newyaw=np.unwrap(new['yaw_world_rad'][300:676]);yawerror=np.abs(newyaw-newyaw[0]-oldyaw+oldyaw[0]);bankerror=np.rad2deg(np.abs(np.array([bank_from_quaternion(q) for q in new['root_quaternion_world_wxyz'][300:676]])-np.array([bank_from_quaternion(q) for q in old['root_quaternion_world_wxyz'][300:676]])))
rows=[]
for h in protocol['prediction_gate']['horizons_s']:
    i=round(h*50);rows.append(dict(horizon_s=h,xy_error_m=float(xyerror[i]),yaw_delta_error_rad=float(yawerror[i]),bank_error_deg=float(bankerror[i])))
stopa=out['development']['first_stop_qualification'];stopb=out['fresh']['first_stop_qualification'];gated=all(r['xy_error_m']<=.1 and r['yaw_delta_error_rad']<=.15 and r['bank_error_deg']<=5 for r in rows) and stopa is not None and stopb is not None and abs(stopa['time_s']-stopb['time_s'])<=.3
repeat=next((d for d in decisions if d['frame']==135),None);repeatpass=repeat is not None and repeat['result']['selected'] is not None
if repeat is not None and not repeatpass:assert not req[135:].any()
completion=json.loads((fresh/'completion_records.json').read_text());last=completion[-1];assert last['executed_blocks']==75
if repeat is not None:assert last['decision_boundary'] and last['actual_stop_qualified']
out.update(protocol_before_dev_utc=protocol['frozen_at_utc'],fresh_definition_before_model_construction=True,actual_decision_checks=checks,
    first_whole_response_validation=dict(rows=rows,xy_rms_m=float(np.sqrt(np.mean(xyerror**2))),xy_peak_m=float(xyerror.max()),whole_yaw_delta_peak_rad=float(yawerror.max()),whole_bank_error_peak_deg=float(bankerror.max()),pre_frozen_gate_passed=bool(gated),scope='single short family at matched same-openplane initial state; yaw/bank are supplemental old response table comparisons'),
    repeat_execution=dict(boundary_really_occurred=repeat is not None,completion=last,actual_repeat_plan_executed=bool(repeatpass),second_result=None if repeat is None else repeat['result'],zero_fallback_is_not_success=not repeatpass,
        actual_xy_displacement_after_boundary_m=float(np.linalg.norm(new['position_world_xy'][-1]-new['position_world_xy'][675])),actual_final_goal_distance_m=float(np.linalg.norm(new['position_world_xy'][-1]-first['goal_world'])),goal_distance_at_first_decision_m=float(np.linalg.norm(new['position_world_xy'][300]-first['goal_world'])),goal_distance_at_plan_end_m=float(np.linalg.norm(new['position_world_xy'][675]-first['goal_world']))),
    no_formal_navigation_or_RGB_scores=True,verdict='accept narrowly validated short matching-stop response; repeat execution only accepted if gate/action/progress pass. Never relax after fresh results.')
(H/'short_branch_assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,2,figsize=(12,8));devxy=old['position_world_xy'][300:676]-old['position_world_xy'][300];freshxy=new['position_world_xy'][300:]-new['position_world_xy'][300];tt=new['simulation_time_s'][300:]-6
axes[0,0].plot(devxy[:,0],devxy[:,1],label='dev 1.5s turn + matched stop');axes[0,0].plot(freshxy[:,0],freshxy[:,1],label='fresh; terminal fallback after second gate');axes[0,0].axis('equal');axes[0,0].legend(fontsize=8);axes[0,0].set_title('Actual XY displacement from first selection')
axes[0,1].plot(np.arange(len(xyerror))*.02,xyerror);axes[0,1].axhline(.1,color='r',ls='--');axes[0,1].set_title('First matching whole response error (m)')
axes[1,0].plot(tt,np.linalg.norm(new['linear_velocity_world_m_s'][300:,:2],axis=1));axes[1,0].axhline(.05,color='r',ls='--');axes[1,0].axvline(1.5,color='k',ls='--');axes[1,0].axvline(7.5,color='k',ls=':');axes[1,0].set_title('Actual XY speed; zero request 1.5s, new selection 7.5s')
axes[1,1].plot(tt,np.rad2deg([bank_from_quaternion(q) for q in new['root_quaternion_world_wxyz'][300:]]));axes[1,1].axvline(1.5,color='k',ls='--');axes[1,1].axvline(7.5,color='k',ls=':');axes[1,1].set_title('Measured bank (deg); no internal reset')
for ax in axes.flat:ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(H/'figures/short_branch_validation.png',dpi=150);plt.close(fig)
