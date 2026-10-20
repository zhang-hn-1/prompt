"""One bounded complete factual run attribution; no alternate requests or physical labels."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;D=H/'original_E1_repeated_exposure_memory_navigation_run_v1';O=H/'original_E1_repeated_exposure_memory_failure_segments_v1';O.mkdir(exist_ok=False)
z=np.load(D/'physics.npz');rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());source=json.loads((D/'source_rows.json').read_text());t=z['simulation_time_s'];p=z['root_position_world_m'];v=z['linear_velocity_world_m_s'];yaw=z['yaw_world_rad'];speed=np.linalg.norm(v[:,:2],axis=1);axis=p[:,0]-(-3.2999999940395353);goal=np.array([3.300000005960465,5.960465010446114e-9]);goalerror=np.linalg.norm(p[:,:2]-goal,axis=1);request=np.load(D/'requests.npz')['requests'];N=len(request);assert len(t)==5*N+1 and len(states)==N+1
phase=[];cuts=[0]
for i in range(1,len(rows)):
 before=rows[i-1]['tracker_state_after_act'];after=rows[i]['tracker_state_after_act']
 if after['active_direction']!=before['active_direction'] or (rows[i-1]['time_s']<6.-1e-6<=rows[i]['time_s']):cuts.append(i)
cuts.append(N)
for lo,hi in zip(cuts[:-1],cuts[1:]):
 a,b=5*lo,5*hi;segment=p[a:b+1];direction=rows[lo]['tracker_state_after_act']['active_direction'];headingdiff=(np.arctan2(v[a:b+1,1],v[a:b+1,0])-yaw[a:b+1]+np.pi)%(2*np.pi)-np.pi
 motion_heading_difference=(headingdiff-(np.pi if direction=='reverse' else 0)+np.pi)%(2*np.pi)-np.pi
 phase.append(dict(course_minus_current_drive_axis_p90_deg=float(np.percentile(np.abs(np.degrees(motion_heading_difference[speed[a:b+1]>.05])),90)) if (speed[a:b+1]>.05).any() else None,start_frame=lo,end_frame=hi,start_time_s=float(t[a]),end_time_s=float(t[b]),direction=direction,axis_progress_delta_m=float(axis[b]-axis[a]),path_length_m=float(np.linalg.norm(np.diff(segment[:,:2],axis=0),axis=1).sum()),goal_error_entry_m=float(goalerror[a]),goal_error_exit_m=float(goalerror[b]),mean_XY_speed_m_s=float(speed[a:b+1].mean()),course_minus_body_yaw_p90_deg=float(np.percentile(np.abs(np.degrees(headingdiff[speed[a:b+1]>.05])),90)) if (speed[a:b+1]>.05).any() else None))
windows=[]
for lo in np.arange(0,t[-1]+1e-6,5):
 mask=(t>=lo)&(t<lo+5);ri=[r for r in rows if lo<=r['time_s']<lo+5];si=[r for r in source if lo<=r['time_s']<lo+5]
 if not mask.any():continue
 windows.append(dict(start_s=float(lo),end_s=float(min(lo+5,t[-1])),true_axis_progress_first_m=float(axis[mask][0]),true_axis_progress_last_m=float(axis[mask][-1]),goal_error_last_m=float(goalerror[mask][-1]),source_exposures=len(si),empty_exposures=sum(r['points']==0 for r in si),actor_frames_no_point_memory=sum(r['raw_point_memory_current']+r['raw_point_memory_past']==0 for r in ri),new_reference_installs=sum(r['local_plan_created'] and r['plan'].get('status')=='success' for r in ri),retained_reference_calls=sum(r['reference_decision'] is not None and r['reference_decision']['decision']=='retain' for r in ri)))
# Old installed geometry evaluated at its next installation BEFORE geometry replacement;
# diagnostic window error not artificially reset by a new starting waypoint.
installs=[i for i,r in enumerate(rows) if r['local_plan_created'] and r['plan'].get('status')=='success' and r['installed_reference_xy_after_act'] is not None];frozen=[]
def distance(point,ref):
 a=ref[:-1];delta=np.diff(ref,axis=0);d=np.einsum('ij,ij->i',delta,delta);u=np.divide(np.einsum('ij,ij->i',point-a,delta),d,out=np.zeros(len(d)),where=d>0);u=np.clip(u,0,1);return float(np.linalg.norm(point-(a+u[:,None]*delta),axis=1).min())
for j,start in enumerate(installs):
 end=installs[j+1] if j+1<len(installs) else N;ref=np.asarray(rows[start]['installed_reference_xy_after_act']);positions=np.asarray([r['position'][:2] for r in states[start:end+1]]);e=np.asarray([distance(x,ref) for x in positions]);frozen.append(dict(installed_frame=start,exit_frame=end,reference_version=rows[start]['local_reference_version'],duration_s=float(states[end]['time_s']-states[start]['time_s']),enter_error_m=float(e[0]),exit_error_m=float(e[-1]),rms_m=float(np.sqrt(np.mean(e**2))),peak_m=float(e.max()),actual_path_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum())))
r=dict(decision='FACTUAL_SEGMENT_AND_REFERENCE_CONSUMPTION_AUDIT_ONLY',budget_s=120.,actual_end_s=float(t[-1]),true_axis_progress_m=float(axis[-1]),axis_peak_progress_m=float(axis.max()),final_goal_error_m=float(goalerror[-1]),physical_path_m=float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum()),direction_segments=len(phase),new_reference_installs=len(installs),rebuild_attempts=sum(r['local_plan_created'] for r in rows),failed_rebuilds=sum(r['local_plan_created'] and r['plan'].get('status')!='success' for r in rows),nonempty_source_exposures=sum(r['points']>0 for r in source),total_source_exposures=len(source),zero_point_memory_action_frames=sum(r['raw_point_memory_current']+r['raw_point_memory_past']==0 for r in rows),no_tuning_or_other_simulation=True,not_hidden_scene_actor=True,no_alternate_plan_truth_label=True,not_navigation_success=True,whole_research_complete=False)
for name,data in [('assessment',r),('direction_segments',phase),('five_second_windows',windows),('installed_reference_exit_errors',frozen)]:(O/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(r,indent=2))
