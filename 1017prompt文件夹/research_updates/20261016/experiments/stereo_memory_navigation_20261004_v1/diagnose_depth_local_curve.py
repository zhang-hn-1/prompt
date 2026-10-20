"""Bounded post-run descriptive failure audit. No fitted/control changes or rerun."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from route_geometry_audit import project
H=Path(__file__).resolve().parent;O=H/'depth_local_curve_run'
r=json.loads((O/'policy_records.json').read_text());a=np.load(O/'episode_arrays.npz');z=json.loads((H/'depth_local_curve_assessment.json').read_text());v=json.loads((H/'depth_local_curve_verification.json').read_text());assert v['interface_gate_pass']
xy=a['positions_world_xy'];t=a['simulation_time_s'];ref=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2];progress,error,_=project(xy,ref)
plans=z['planning']['rows'];targets=np.asarray([x['local_target'] for x in plans]);side=np.sign(targets[:,1]);switches=[]
for i in range(1,len(plans)):
 if side[i]!=side[i-1]:
  f=plans[i]['frame'];switches.append(dict(frame=f,time_s=float(t[f]),old_target=targets[i-1].tolist(),new_target=targets[i].tolist(),old_current_hazard=r[plans[i-1]['frame']]['local']['current_occupied'],new_current_hazard=r[f]['local']['current_occupied'],old_historical_hazard=r[plans[i-1]['frame']]['local']['historical_occupied'],new_historical_hazard=r[f]['local']['historical_occupied']))
phases=[]
for name,lo,hi in [('zero_prefix',0,60),('forward_tracking',60,160),('direction_braking',160,185),('reverse_tracking',185,350)]:
 rr=r[lo:hi];valid=[x for x in rr if x['tracking'].get('valid')];phases.append(dict(name=name,start_frame=lo,end_frame=hi,start_s=float(t[lo]),end_s=float(t[hi]),actual_path_m=float(np.linalg.norm(np.diff(xy[lo:hi+1],axis=0),axis=1).sum()),reference_progress_delta_m=float(progress[hi]-progress[lo]),entry_lateral_m=float(error[lo]),exit_lateral_m=float(error[hi]),lateral_rms_m=float(np.sqrt(np.mean(error[lo:hi+1]**2))),tracking_valid_count=len(valid),protocol_clipped_count=sum(bool(x['tracking'].get('protocol_clipped')) for x in valid),bank_curvature_clamped_count=sum(bool(x['tracking'].get('calibrated_bank_mapping',{}).get('curvature_clamped')) for x in valid),v1_outside_count=sum(bool(x['tracking'].get('requested_curvature_outside_v1_support')) for x in valid)))
active=a['policy_active_drive_direction'];desired=a['policy_desired_drive_direction'];changes=lambda arr:[dict(frame=int(i),time_s=float(t[i]),before=float(arr[i-1]),after=float(arr[i])) for i in np.flatnonzero(np.diff(arr)!=0)+1]
# Fixed references measured after installation (not reprojecting every new path).
persistent=[]
for f in [60,100,150,160,200]:
 local=np.load(O/'local_plans'/f'frame_{f:03d}.npz')['pose'][:,:2];end=min(350,f+50);pr,er,_=project(xy[f:end+1],local);persistent.append(dict(start_frame=f,end_frame=end,frozen_reference_progress_delta_m=float(pr[-1]-pr[0]),exit_error_m=float(er[-1]),peak_error_m=float(er.max()),scope='descriptive five-second comparison against that installed path, superseded online; not an intervention or tracking-only ground truth'))
res=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_assessment_sha256=hashlib.sha256((H/'depth_local_curve_assessment.json').read_bytes()).hexdigest(),input_verification_sha256=hashlib.sha256((H/'depth_local_curve_verification.json').read_bytes()).hexdigest(),local_target_world_y_sign_changes=switches,active_direction_changes=changes(active),desired_direction_changes=changes(desired),phases=phases,frozen_reference_descriptive_windows=persistent,half_second_window_exit_rms_m=float(np.sqrt(np.mean([x['exit_error_m']**2 for x in z['planning']['windows']]))),half_second_window_peak_m=max(x['peak_error_m'] for x in z['planning']['windows']),attribution='Changes in proposed side and sustained clipping are observed. Their individual causal contributions to timeout are not isolated. Only one actual direction reversal; no direction thrash conclusion. Short reinitialized reference errors do not establish persistent progress.',whole_research_complete=False)
(H/'depth_local_curve_failure_audit.json').write_text(json.dumps(res,indent=2)+'\n')
fig,ax=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
o=np.load(O/'offline_occupancy_label.npz');ii=np.argwhere(o['labels']==1);ww=o['origin']+ii*float(o['resolution']);ax[0,0].scatter(ww[:,0],ww[:,1],s=3,c='lightgray',label='Offline obstacle label (evaluation only)');ax[0,0].plot(ref[:,0],ref[:,1],'--',c='gray',label='Frozen Oracle reference (evaluation only)');ax[0,0].plot(xy[:,0],xy[:,1],c='black',lw=2,label='Actual local navigation');ax[0,0].scatter(xy[0,0],xy[0,1],c='blue',marker='o',label='Start');ax[0,0].scatter(3.3,0,c='green',marker='*',s=120,label='Goal')
for f,c in [(60,'tab:blue'),(100,'tab:orange'),(160,'tab:red'),(300,'tab:purple')]:
 local=np.load(O/'local_plans'/f'frame_{f:03d}.npz')['pose'][:,:2];ax[0,0].plot(local[:,0],local[:,1],c=c,alpha=.8,label=f'Installed local path at {t[f]:.1f}s')
ax[0,0].axis('equal');ax[0,0].set(xlabel='World x (m)',ylabel='World y (m)',title='35s timeout: 4.487m remains');ax[0,0].legend(fontsize=7,loc='upper left')
ax[0,1].step([x['time_s'] for x in plans],targets[:,1],where='post',label='Local target world y');ax[0,1].plot(t,xy[:,1],label='Actual world y');ax[0,1].axhline(0,c='gray',lw=.5);ax[0,1].set(xlabel='Physics time (s)',ylabel='World y (m)',title='Local targets change side (descriptive)');ax[0,1].legend()
k=np.array([x['tracking'].get('requested_curvature_1_m',np.nan) if x['tracking'].get('valid') else np.nan for x in r]);ax[1,0].plot(t[:-1],k,label='Requested curvature');clip=[i for i,x in enumerate(r) if x['tracking'].get('valid') and x['tracking'].get('protocol_clipped')];ax[1,0].scatter(t[clip],k[clip],s=9,c='red',label='Protocol clipped');ax[1,0].set(xlabel='Physics time (s)',ylabel='Curvature (1/m)',title='186 / 265 moving tracking requests clipped');ax[1,0].legend(fontsize=8)
ax[1,1].plot(t,progress,label='Progress on frozen evaluation reference (m)');ax[1,1].plot(t,error,label='Lateral distance to same reference (m)');ax[1,1].step(t[:-1],active,where='post',label='Active direction (+1 / -1)',alpha=.6);ax[1,1].axvspan(16,18.5,color='orange',alpha=.15,label='Actual original braking');ax[1,1].set(xlabel='Physics time (s)',title='Independent geometry; one actual reversal');ax[1,1].legend(fontsize=8)
fig.suptitle('Privileged renderer depth + known pose; no global map/reference input; not RGB navigation',fontsize=12)
fig.savefig(H/'depth_local_curve_failure.png',dpi=160);plt.close(fig)
print(json.dumps({k:w for k,w in res.items() if k!='frozen_reference_descriptive_windows'},indent=2))
