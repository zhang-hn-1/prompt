"""Factual FIRST given pause only; no new controller, prediction or event selection."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'given_program_first_pause_cause_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
g=json.loads((H/'given_raw_request_program_v1.json').read_text());first=g['modes'].index('GIVEN_PROGRAM_GOAL_PAUSE');end=first
while end<len(g['modes']) and g['modes'][end]=='GIVEN_PROGRAM_GOAL_PAUSE':end+=1
assert first==555 and end==557
protocol=dict(scope='FIRST_PROGRAM_PAUSE_FACTUAL_BOTH_NEW_ACTUALS',first_frame=first,end_frame_exclusive=end,window_s=[55.5,57.0],no_new_action_or_event_pick=True,inputs={str(H/('bank_hold_request_program_'+a+'_run_v1')/f):sha(H/('bank_hold_request_program_'+a+'_run_v1')/f) for a in ['B0','C'] for f in ['physics.npz','policy_records.json','feedback/bank_feedback_controlled_static_obstacle_v1.json']})
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
result={};fig,ax=plt.subplots(4,1,figsize=(10,10),sharex=True)
for arm in ['B0','C']:
 D=H/('bank_hold_request_program_'+arm+'_run_v1');p=np.load(D/'physics.npz');f=json.loads((D/'feedback/bank_feedback_controlled_static_obstacle_v1.json').read_text())
 lo,hi=first*5,first*5+75;t=p['simulation_time_s'][lo:hi+1];pos=p['root_position_world_m'][lo:hi+1];speed=np.linalg.norm(p['linear_velocity_world_m_s'][lo:hi+1,:2],axis=1);ap=p['applied_action'][lo:hi+1,0];sm=p['smoothed_target'][lo:hi+1,0];fb=f[lo:hi];bank=np.array([v['bank_rad'] for v in fb]);desired=np.array([v['desired_bank_rad'] for v in fb]);dist=np.linalg.norm(pos[:,:2]-[2.2,.15],axis=1)
 requests=np.asarray([r['request'] for r in json.loads((D/'policy_records.json').read_text())])[first:first+15].reshape(15,5,2)
 # Five 10 Hz speed confirmations, not joint zero or 50 Hz samples.
 speeds10=speed[::5];count=0;qualified=[]
 for j,v in enumerate(speeds10):
  count=count+1 if v<.05 else 0
  if count>=5:qualified.append(float(t[j*5]))
 result[arm]=dict(entry_speed_XY_m_s=float(speed[0]),exit_given_pause_speed_XY_m_s=float(speed[10]),entry_applied_main=float(ap[0]),minimum_applied_main=float(ap.min()),minimum_smoothed_main=float(sm.min()),applied_main_zero_any=bool(np.any(ap==0)),minimum_speed_XY_m_s=float(speed.min()),five_10Hz_low_speed_qualification_s=qualified,entry_actual_bank_deg=float(np.degrees(bank[0])),pause_bank_error_peak_deg=float(np.degrees(abs(desired[:10]-bank[:10])).max()),pause_desired_bank_start_deg=float(np.degrees(desired[0])),pause_desired_bank_end_deg=float(np.degrees(desired[9])),next_tracking_desired_bank_deg=float(np.degrees(desired[10])),radial_peak_1p5s_m=float(np.linalg.norm(pos[:,:2]-pos[0,:2],axis=1).max()),closest_XY_window_m=float(dist.min()),requested_main_at_resume=float(requests[2,0,0]),supervisor_active_pause_ticks=sum(v['supervisor_active'] for v in fb[:10]),pid_clip_pause_ticks=sum(v['pid_clipped'] for v in fb[:10]))
 ax[0].plot(t,ap,label=arm+' applied main');ax[0].plot(t,sm,ls='--',alpha=.5,label=arm+' smooth main')
 ax[1].plot(t,speed,label=arm);ax[2].plot(t[:-1],np.degrees(bank),label=arm+' actual');ax[2].plot(t[:-1],np.degrees(desired),ls='--',label=arm+' desired');ax[3].plot(t,dist,label=arm)
for a in ax:a.axvspan(55.5,55.7,color='gray',alpha=.15);a.legend(fontsize=8)
ax[0].set_ylabel('Main (rad/s)');ax[1].set_ylabel('XY speed (m/s)');ax[1].axhline(.05,color='gray',ls=':');ax[2].set_ylabel('Bank (deg)');ax[3].set_ylabel('True goal XY (m)');ax[3].axhline(.2,color='gray',ls=':');ax[3].set_xlabel('Actual time (s)');fig.suptitle('FIRST given pause: physical action intervention, no navigation claim');fig.tight_layout();fig.savefig(O/'first_pause_physics.png',dpi=150)
(O/'assessment.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pause_request_duration_s=.2,arms=result,no_new_controller_candidate=True,no_global_stopping_guarantee=True,no_claim_alternate_longer_pause_would_succeed=True),indent=2)+'\n');print(json.dumps(result,indent=2))
