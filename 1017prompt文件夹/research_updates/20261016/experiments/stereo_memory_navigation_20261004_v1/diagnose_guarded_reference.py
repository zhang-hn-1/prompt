"""Bounded descriptive segmentation of the unique completed physical run."""
from pathlib import Path
import json,datetime
import numpy as np
from route_geometry_audit import project
H=Path(__file__).resolve().parent;O=H/'guarded_reference_run'
def main():
 r=json.loads((O/'policy_records.json').read_text());a=np.load(O/'episode_arrays.npz');states=json.loads((O/'states.json').read_text());ref=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2];progress,error,_=project(a['positions_world_xy'],ref)
 source=[x['request']['request_source'] for x in r];changes=[0]+[i for i in range(1,len(r)) if source[i]!=source[i-1]]+[len(r)];segments=[]
 for start,end in zip(changes[:-1],changes[1:]):
  xy=a['positions_world_xy'][start:end+1]
  segments.append(dict(start_frame=start,end_frame=end,start_time_s=float(r[start]['local']['time_s']),end_time_s=float(states[end]['time_s']),source=source[start],duration_s=(end-start)/10.,actual_path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),independent_progress_gain_m=float(progress[end]-progress[start]),entry_error_m=float(error[start]),exit_error_m=float(error[end]),desired_directions=sorted(set(x['diagnostics']['desired_drive_direction'] for x in r[start:end])),active_directions=sorted(set(x['diagnostics']['active_drive_direction'] for x in r[start:end])),maximum_low_count=max(x['diagnostics']['stopped_cycles'] for x in r[start:end])))
 versions=[]
 for x in r:
  if not x['local']['local_plan_created']:continue
  i=x['frame'];local=x['local'];plan=local['plan'];pathfile=O/'local_plans'/f'frame_{i:03d}.npz';entry=dict(frame=i,time_s=local['time_s'],status=plan['status'],reference_version=local['local_reference_version'],active_direction=x['diagnostics']['active_drive_direction'],pending_direction=x['diagnostics']['pending_drive_direction'],low_count=x['diagnostics']['stopped_cycles'],guard_reasons=[] if local['reference_decision']['guard'] is None else local['reference_decision']['guard']['reject_reasons'])
  if pathfile.exists():
   path=np.load(pathfile);entry.update(initial_reference_direction=int(path['direction'][0]),path_direction_changes=int((np.diff(path['direction'])!=0).sum()),local_target=plan['local_target'],path_length_m=plan['local_path_length_m'])
  versions.append(entry)
 transitions=[dict(frame=i,time_s=x['local']['time_s'],active=x['diagnostics']['active_drive_direction'],desired=x['diagnostics']['desired_drive_direction'],source=x['request']['request_source']) for i,x in enumerate(r) if i and x['diagnostics']['active_drive_direction']!=r[i-1]['diagnostics']['active_drive_direction']]
 result=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),segments=segments,reference_rebuilds=versions,actual_policy_direction_transitions=transitions,scope='Actual saved request-source contiguous phases and reference changes; no alternate physical truth, no control/model admission.',whole_research_complete=False)
 (H/'guarded_reference_failure_segments.json').write_text(json.dumps(result,indent=2)+'\n')
 for x in segments:print(json.dumps(x))
 print('DIRECTION',json.dumps(transitions));print('REBUILDS',json.dumps(versions))
if __name__=='__main__':main()
