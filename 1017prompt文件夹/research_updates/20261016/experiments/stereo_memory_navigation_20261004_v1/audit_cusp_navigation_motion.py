"""Closed actual sequence only: acceptance, executed direction and known segment geometry."""
from pathlib import Path
import json, datetime
import numpy as np
from route_geometry_audit import project
H=Path(__file__).resolve().parent; O=H/'cusp_navigation_run'

def main():
 status=json.loads((H/'cusp_navigation_status.json').read_text());assert status['status']=='finished' and status['exit_code']==0
 records=json.loads((O/'policy_records.json').read_text());a=np.load(O/'episode_arrays.npz');ph=np.load(O/'physics.npz');N=len(records)
 speed=np.linalg.norm(a['linear_velocity_world_m_s'][:,:2],axis=1);transitions=[];previous='forward'
 for i,r in enumerate(records):
  d=r['diagnostics'];accepted=d['active_drive_direction']
  if accepted!=previous:
   inds=list(range(max(0,i-4),i+1));lows=speed[inds]<.05
   transitions.append(dict(frame=i,time_s=i/10.,from_direction=previous,to_direction=accepted,
     actual_xy_speed_m_s=float(speed[i]),last_five_current_speeds_m_s=speed[inds].tolist(),
     five_low_independent_check=bool(len(inds)==5 and lows.all()),
     reference_index=d['reference_index'],reference_version=r['local']['local_reference_version'],
     pending_after=d['pending_drive_direction'],request_source=r['request']['request_source'],
     raw_main=r['request']['policy_raw_main_command_rad_s']))
  previous=accepted
 phases=[]
 for i,r in enumerate(records):
  key=(r['request']['request_source'],r['diagnostics']['active_drive_direction'])
  if not phases or phases[-1]['key']!=list(key):phases.append(dict(key=list(key),start_frame=i,end_frame=i+1))
  else:phases[-1]['end_frame']=i+1
 frozen=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2];pr,er,_=project(a['positions_world_xy'],frozen)
 for row in phases:
  i,j=row['start_frame'],row['end_frame'];row.update(start_time_s=i/10.,end_time_s=j/10.,duration_s=(j-i)/10.,independent_progress_gain_m=float(pr[j]-pr[i]),entry_error_m=float(er[i]),exit_error_m=float(er[j]),peak_error_m=float(er[i:j+1].max()))
 segments=[]
 # Each installed reference is known at installation; evaluate only its actual lifespan.
 for i,r in enumerate(records):
  if not r['local']['local_plan_created'] or r['local']['plan']['status']!='success':continue
  ref=np.load(O/'local_plans'/f'frame_{i:03d}.npz');version=int(ref['version']);end=next((k for k in range(i+1,N) if records[k]['local']['local_reference_version']!=version),N)
  xy=a['positions_world_xy'][i:end+1];dirs=ref['direction'];poses=ref['pose']
  for k in np.flatnonzero(dirs[:-1]<0):
   start,stop=poses[k,:2],poses[k+1,:2];delta=stop-start;length=float(np.linalg.norm(delta))
   if length<=1e-12:continue
   signed=(xy-start)@(delta/length)
   startdist=np.linalg.norm(xy-start,axis=1);enddist=np.linalg.norm(xy-stop,axis=1)
   negative=[j for j in range(i,end) if records[j]['request']['policy_raw_main_command_rad_s']<0]
   ticks=slice(5*i+1,5*end+1);raw=ph['raw_command_target'][ticks,0];applied=ph['applied_action'][ticks,0];joint=ph['joint_velocity_rad_s'][ticks,0]
   vel=a['linear_velocity_world_m_s'][i:end+1,:2];along=vel@(delta/length)
   segments.append(dict(installed_frame=i,reference_version=version,observed_until_frame=end,primitive_start_index=int(k),start_xy=start.tolist(),end_xy=stop.tolist(),nominal_length_m=length,
    minimum_distance_to_start_m=float(startdist.min()),minimum_distance_to_end_m=float(enddist.min()),
    signed_progress_min_m=float(signed.min()),signed_progress_max_m=float(signed.max()),signed_progress_end_m=float(signed[-1]),
    maximum_actual_velocity_along_segment_m_s=float(along.max()),minimum_actual_velocity_along_segment_m_s=float(along.min()),
    negative_request_frames=negative,negative_raw_ticks=int((raw<0).sum()),negative_applied_ticks=int((applied<0).sum()),negative_main_joint_ticks=int((joint<0).sum()),
    reference_replaced_at_end=end<N,scope='Continuous descriptive geometry over own installed lifespan; no new success threshold/alternate-plan physical label.'))
 result=dict(audited_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),accepted_transitions=transitions,actual_request_direction_phases=phases,known_reverse_primitives=segments,
  all_acceptances_have_five_current_low=all(t['five_low_independent_check'] for t in transitions),
  whole_run_negative_raw_ticks=int((ph['raw_command_target'][1:,0]<0).sum()),whole_run_negative_applied_ticks=int((ph['applied_action'][1:,0]<0).sum()),whole_run_negative_main_joint_ticks=int((ph['joint_velocity_rad_s'][1:,0]<0).sum()),
  current_request_vs_actual_motion_separate=True,offline_only=True,no_new_control_or_gate=True,whole_research_complete=False)
 (H/'cusp_navigation_motion_audit.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))

if __name__=='__main__':main()
