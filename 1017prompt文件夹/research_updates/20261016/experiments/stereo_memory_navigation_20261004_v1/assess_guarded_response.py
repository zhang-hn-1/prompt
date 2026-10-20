"""Independent whole-sequence audit; never imported by the live consumer."""
import hashlib,json,subprocess
from pathlib import Path
import numpy as np
from guarded_response import GuardedResponseConsumer,applicability,physical_state
from execution_snapshot import ExecutionClone
from guarded_decision_replay import replay_memory
from motion_measurement import qualification
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent;r=H/'guarded_response_run';p=np.load(r/'physics.npz');d=json.loads((r/'decision.json').read_text());proto=json.loads((H/'guarded_response_protocol.json').read_text())
entries=json.loads((H/'guarded_response_library.json').read_text())
for e in entries:
    for k in ('xy','requests','times'):e[k]=np.asarray(e[k])
consumer=GuardedResponseConsumer(entries)
files=sorted((r/'packets').glob('frame_*.npz'))
def loader(f):
    with np.load(f) as x:return {k:x[k] for k in x.files}
results=[]
for paths in (files,files[:61],files[:61]+[r/'future_poison_packet_DO_NOT_READ.npz']):
    memory=replay_memory(paths,60,loader)
    result=consumer.select(d['current'],d['execution'],d['goal_world'],memory.labels(memory.time),memory.origin,memory.resolution)
    results.append(result)
for result in results:
    assert result['selected']==d['result']['selected']
    np.testing.assert_array_equal(result['world_xy'],d['result']['world_xy'])
    np.testing.assert_array_equal(result['requests'],d['result']['requests'])
mapfile=np.load(r/'decision_map.npz')
np.testing.assert_array_equal(memory.labels(memory.time),mapfile['labels'])
np.testing.assert_array_equal(memory.floor_time,mapfile['provenance_floor_time'])
np.testing.assert_array_equal(memory.hazard_time,mapfile['provenance_hazard_time'])
requests=np.load(r/'requests.npz')['requests'];np.testing.assert_array_equal(p['raw_command_target'][1:],requests.reshape(-1,2))
assert not requests[:60].any();np.testing.assert_array_equal(requests[60:],results[0]['requests'])
clocks=json.loads((r/'camera_clocks.json').read_text());states=json.loads((r/'execution_states.json').read_text())
assert len(clocks)==181 and len(p['simulation_time_s'])==901 and len(files)==181 and len(states)==181
assert states[0] is None and states[60]==d['execution']
# Clone exported live state and replay measured subsequent feedback, independently.
clone=ExecutionClone.from_snapshot(states[60]);maxerror=0.;snapshot_exact=True
for tick in range(300,900):
    a,s=clone.step(p['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(p['root_quaternion_world_wxyz'][tick]),aux_position_rad=p['joint_position_rad'][tick,1],aux_velocity_rad_s=p['joint_velocity_rad_s'][tick,1])
    maxerror=max(maxerror,float(np.max(abs(a-p['applied_action'][tick+1]))),float(np.max(abs(s-p['smoothed_target'][tick+1]))))
    if (tick+1)%5==0:snapshot_exact&=clone.snapshot()==states[(tick+1)//5]
assert maxerror==0 and snapshot_exact
# Do not score an alternative plan against the actual plan's label.
assert results[0]['selected']=='positive';old=np.load(H/'large_bank_runs/positive/physics.npz');i=300
pred=np.asarray(d['result']['world_xy']);actual=p['position_world_xy'][i:];error=np.linalg.norm(actual-pred,axis=1)
oldyaw=np.unwrap(old['yaw_world_rad'][i:]);newyaw=np.unwrap(p['yaw_world_rad'][i:]);yawerror=np.abs((newyaw-newyaw[0])-(oldyaw-oldyaw[0]))
oldbank=np.array([bank_from_quaternion(q) for q in old['root_quaternion_world_wxyz'][i:]])
newbank=np.array([bank_from_quaternion(q) for q in p['root_quaternion_world_wxyz'][i:]])
bankerror=np.rad2deg(np.abs(newbank-oldbank))
rows=[]
for horizon in proto['prediction_gate']['horizons_s']:
    j=round(horizon*50)
    rows.append(dict(horizon_s=horizon,xy_error_m=float(error[j]),yaw_delta_error_rad=float(yawerror[j]),bank_error_deg=float(bankerror[j]),actual_xy_world=actual[j].tolist(),predicted_xy_world=pred[j].tolist()))
def firststop(trace):
    speed=np.linalg.norm(trace['linear_velocity_world_m_s'][::5,:2],axis=1);q,counts=qualification(speed)
    after=np.flatnonzero(q & (trace['simulation_time_s'][::5]>=12-1e-5))
    return None if not len(after) else float(trace['simulation_time_s'][::5][after[0]])
oldstop=firststop(old);newstop=firststop(p)
gate=all(a['xy_error_m']<=.1 and a['yaw_delta_error_rad']<=.15 and a['bank_error_deg']<=5 for a in rows) and oldstop is not None and newstop is not None and abs(newstop-oldstop)<=.3
# Current historical navigation states are diagnostics, not fresh model labels.
navr=H/'online_provenance_runs/history';navp=np.load(navr/'physics.npz')
# Start from the actual deterministic reset definition; do not import a builder
# that writes development artifacts during independent assessment.
from types import SimpleNamespace
from execution_snapshot import export_execution
from scripts.action_smoothing import ActionSmoother
from training.action_protocol import load_action_protocol
from roll_pid import BankPID
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
initial_execution=export_execution(SimpleNamespace(inner=ActionSmoother(.02),pid=BankPID(.4,.1,.7),blocked=False),load_action_protocol(P/'configs/data_contract.yaml'))
navclone=ExecutionClone.from_snapshot(initial_execution)
accepted=[];target=None;total=0;navmax=0.
for tick in range(len(navp['simulation_time_s'])-1):
    a,s=navclone.step(navp['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(navp['root_quaternion_world_wxyz'][tick]),aux_position_rad=navp['joint_position_rad'][tick,1],aux_velocity_rad_s=navp['joint_velocity_rad_s'][tick,1])
    navmax=max(navmax,float(np.max(abs(a-navp['applied_action'][tick+1]))),float(np.max(abs(s-navp['smoothed_target'][tick+1]))))
    if (tick+1)%5:continue
    n=tick+1;t=float(navp['simulation_time_s'][n])
    if t<6-1e-5:continue
    cur=physical_state(np.r_[navp['position_world_xy'][n],.45],navp['root_quaternion_world_wxyz'][n],navp['linear_velocity_world_m_s'][n],navp['angular_velocity_world_rad_s'][n],navp['joint_position_rad'][n],navp['joint_velocity_rad_s'][n])
    gates=[dict(name=e['name'],**applicability(cur,navclone.snapshot(),e)) for e in entries]
    total+=1
    if any(g['applicable'] for g in gates):accepted.append(t)
    if abs(t-28.4)<1e-4:target=dict(time_s=t,gates=gates)
assert navmax==0
history_checks=0
hist=sorted((r/'history').glob('*.npz'))
for a,b in zip(hist[:-1],hist[1:]):
    with np.load(a) as x,np.load(b) as y:
        for key in x.files:np.testing.assert_array_equal(x[key][1:],y[key][:-1])
    history_checks+=1
identity=json.loads((r/'identity.json').read_text());hashes=True
for f,h in identity['source_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h
for f,h in proto['source_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h
videos={}
for filename in ('stereo_view.mp4','third_person.mp4'):
    command=['ffprobe','-v','error','-count_frames','-show_entries','stream=nb_read_frames,r_frame_rate,duration','-of','json',str(r/filename)]
    videos[filename]=json.loads(subprocess.check_output(command))['streams'][0]
    assert int(videos[filename]['nb_read_frames'])==181
out=dict(role='single fresh whole yaw20 response validation; not navigation/general-state acceptance',selected=d['result']['selected'],support=d['result']['support'],rows=rows,
    measured_response_comparison_scope='online consumer outputs archived XY and known requests; yaw/bank are supplemental historical table comparison, not online recursive outputs',
    xy_whole_rms_m=float(np.sqrt(np.mean(error**2))),xy_whole_peak_m=float(error.max()),yaw_delta_peak_rad=float(yawerror.max()),bank_peak_error_deg=float(bankerror.max()),old_stop_time_s=oldstop,new_stop_time_s=newstop,pre_frozen_whole_gate_passed=bool(gate),
    execution_clone_measured_feedback_exact=True,complete_live_snapshots_exact=True,causal_map_replay_exact=True,future_packet_truncate_replace_unchanged=True,requests_match_selected_and_realphysics=True,
    counts=dict(requests=180,captures=181,physics=901,archived_stereo_pairs=37,packets=181,history_shifts_once=history_checks,complete_live_execution_snapshots=180,initial_capture_before_PID_install=1),videos=videos,all_frozen_hashes_exact=True,
    actual_max_camera_age_s=max(abs(c['raw_time_s']-t) for c in clocks for t in c['eyes'].values()),historical_E1=dict(scope='historical applicability audit only, zplaceholder ignored/no terrain transfer',samples_after6s=total,applicable_times_s=accepted,first_veto_state=target,execution_measured_reconstruction_maxerror=navmax),
    verdict='accept narrow same-openplane cold-state yaw transfer only if gate passes; no navigation/unknown safety or arbitrary state predictor admission',
    exclusions=['no stitching early stop tail','no current-run future feedback input','no measured terrain-height coverage','no safe cold-start certificate','no RGB/estimated-pose/learned increment/formal35s results'])
(H/'guarded_response_assessment.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
# Scientific output, all samples kept.
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,2,figsize=(12,8));tt=p['simulation_time_s'][i:]-6
axes[0,0].plot(pred[:,0],pred[:,1],label='historical response rotated to current yaw');axes[0,0].plot(actual[:,0],actual[:,1],label='fresh actual yaw+20');axes[0,0].axis('equal');axes[0,0].legend(fontsize=8);axes[0,0].set_title('Whole turn + its own stop tail: world XY')
axes[0,1].plot(tt,error);axes[0,1].axhline(.1,color='r',ls='--');axes[0,1].set_title('Position error (m); frozen gate at six horizons')
axes[1,0].plot(tt,np.rad2deg(oldbank),label='old bank');axes[1,0].plot(tt,np.rad2deg(newbank),label='fresh bank');axes[1,0].axvline(6,color='k',ls='--');axes[1,0].legend();axes[1,0].set_title('Actual bank comparison (deg)')
axes[1,1].plot(tt,np.linalg.norm(old['linear_velocity_world_m_s'][i:,:2],axis=1),label='old XY speed');axes[1,1].plot(tt,np.linalg.norm(p['linear_velocity_world_m_s'][i:,:2],axis=1),label='fresh XY speed');axes[1,1].axhline(.05,color='r',ls='--');axes[1,1].axvline(6,color='k',ls='--');axes[1,1].legend();axes[1,1].set_title('Measured stopping / rebound (m/s)')
for ax in axes.flat:ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(H/'figures/guarded_response_validation.png',dpi=150);plt.close(fig)
