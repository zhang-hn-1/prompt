"""Two saved handoff frames, no policy act, plan search or counterfactual physics."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from route_geometry_audit import project
H=Path(__file__).resolve().parent;O=H/'root_direction_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 records=json.loads((O/'policy_records.json').read_text());arrays=np.load(O/'episode_arrays.npz');ph=np.load(O/'physics.npz');ref=np.load(O/'local_plans/frame_175.npz');expected=np.load(H/'direction_root_cost_candidate_reference.npz')
 for k in ['pose','direction','command']:np.testing.assert_equal(ref[k],expected[k])
 rows=[]
 for i in [181,182]:
  previous=records[i-1]['diagnostics'];cur=records[i]['diagnostics'];before=previous['reference_index'];active=previous['active_drive_direction'];sign=1 if active=='forward' else -1;last=len(ref['pose'])-1
  for j in range(before+1,len(ref['pose'])):
   if ref['direction'][j]!=sign:last=j;break
  xy=arrays['positions_world_xy'][i];distance=np.linalg.norm(ref['pose'][before:last+1,:2]-xy,axis=1);selected=before+int(np.argmin(distance));assert selected==cur['reference_index']
  assert not records[i]['local']['local_plan_created'] and records[i]['local']['local_reference_version']==3
  rows.append(dict(frame=i,current_xy=xy.tolist(),before_index=int(before),active_before=active,pending_before=previous['pending_drive_direction'],stopped_cycles_before=previous['stopped_cycles'],reference_direction_at_before='forward' if ref['direction'][before]>0 else 'reverse',allowed_candidate_indices=[int(before),int(last)],distances_m=distance.tolist(),reconstructed_selected_index=int(selected),actual_after=cur,actual_source=records[i]['request']['request_source']))
 i,j=175,230;xy=arrays['positions_world_xy'][i:j+1];localpr,localer,_=project(xy,ref['pose'][:,:2]);vel=arrays['linear_velocity_world_m_s'][i:j+1,:2];yaw=arrays['yaw_world_rad'][i:j+1];long=np.sum(vel*np.c_[np.cos(yaw),np.sin(yaw)],axis=1)
 ticks=slice(5*i+1,5*j+1);raw=ph['raw_command_target'][ticks,0];applied=ph['applied_action'][ticks,0];joint=ph['joint_velocity_rad_s'][ticks,0]
 segments=[]
 for start,end in [(175,230),(230,270),(270,305),(305,350)]:
  frozen=np.load(O/'local_plans'/f'frame_{start:03d}.npz');pr,er,_=project(arrays['positions_world_xy'][start:end+1],frozen['pose'][:,:2]);versions=[r['local']['local_reference_version'] for r in records[start:end]]
  segments.append(dict(start_frame=start,end_frame=end,reference_version=int(frozen['version']),version_changes_within_window=len(set(versions))>1,progress_m=float(pr[-1]-pr[0]),entry_error_m=float(er[0]),exit_error_m=float(er[-1]),lateral_rms_m=float(np.sqrt(np.mean(er**2))),lateral_peak_m=float(er.max()),scope='Frozen installed-reference descriptive window; replacement windows explicitly marked, not another-plan true label.'))
 result=dict(audited_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),nominal175_exact_previous_candidate=True,first_three_primitive_directions=ref['direction'][:3].tolist(),rows=rows,window_17p5_23=dict(raw_negative_ticks=int((raw<0).sum()),applied_negative_ticks=int((applied<0).sum()),main_joint_negative_ticks=int((joint<0).sum()),minimum_raw_main=float(raw.min()),minimum_applied_main=float(applied.min()),minimum_main_joint=float(joint.min()),minimum_root_longitudinal_xy_speed_m_s=float(long.min()),local_reference_progress_m=float(localpr[-1]-localpr[0]),local_reference_lateral_rms_m=float(np.sqrt(np.mean(localer**2))),local_reference_lateral_peak_m=float(localer.max())),frozen_installed_reference_windows=segments,causal_interface_fact='At182 initial projected waypoint is reverse while accepted remains forward; inherited search-window loop starts at next waypoint and admits later forward section before reverse qualification.',reverse_primitive_execution_demonstrated=False,no_new_control_fix=True,no_future_feedback_input=True,offline_only=True,source_inputs={str(f):sha(f) for f in [O/'policy_records.json',O/'episode_arrays.npz',O/'physics.npz',O/'local_plans/frame_175.npz',H/'direction_root_cost_candidate_reference.npz']})
 (H/'root_direction_handoff_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='source_inputs'},indent=2))
if __name__=='__main__':main()
