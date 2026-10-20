"""Author supplemental measurements from saved installed references only."""
from pathlib import Path
import json,numpy as np
from route_geometry_audit import tracking_window
H=Path(__file__).resolve().parent
def measure():
 r=json.loads((H/'guarded_reference_run/policy_records.json').read_text());a=np.load(H/'guarded_reference_run/episode_arrays.npz');starts=[x['frame'] for x in r if x['local']['local_plan_created']]+[len(r)];rows=[]
 for start,end in zip(starts[:-1],starts[1:]):
  f=H/'guarded_reference_run/local_plans'/f'frame_{start:03d}.npz'
  if not f.exists():continue
  path=np.load(f);rows.append(dict(start_frame=start,end_frame=end,duration_s=(end-start)/10.,**tracking_window(a['positions_world_xy'][start:end+1],path['pose'][:,:2])))
 active_rows=[]
 for start,end in [(60,175),(175,230),(260,350)]:
  active=[x for x in r[start:end] if x['tracking'].get('valid') and x['request']['request_source']=='tracking'];main,counts=np.unique([x['request']['policy_protocol_main_command_rad_s'] for x in active],return_counts=True)
  active_rows.append(dict(start_frame=start,end_frame=end,moving_valid_frames=len(active),actual_speed_mean_m_s=float(np.mean([x['tracking']['world_xy_speed_m_s'] for x in active])),protocol_clipped_fraction=float(np.mean([x['tracking']['protocol_clipped'] for x in active])),main_commands=main.tolist(),main_counts=counts.tolist()))
 return rows,active_rows
if __name__=='__main__':
 windows,active=measure();f=H/'guarded_reference_failure_segments.json';j=json.loads(f.read_text());assert j['actual_installed_reference_windows']==windows
 # Existing window measurement is preserved; add the separately printed actual moving-frame data.
 j['actual_moving_tracking_groups']=active;f.write_text(json.dumps(j,indent=2)+'\n');print(json.dumps(active,indent=2))
