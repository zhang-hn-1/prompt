"""Postrun descriptive measurement only; no policy/fit/alternate plan labels."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
H=Path(__file__).resolve().parent;O=H/'endpoint_navigation_run'
def main():
 status=json.loads((H/'endpoint_navigation_status.json').read_text());assert status['exit_code']==0
 records=json.loads((O/'policy_records.json').read_text());a=np.load(O/'episode_arrays.npz');ph=np.load(O/'physics.npz');fb=json.loads((O/'feedback/bank_feedback_golden_e1_0001.json').read_text());motion=json.loads((H/'endpoint_navigation_motion_audit.json').read_text())
 assert len(fb)==len(records)*5
 rows=[]
 for i,r in enumerate(records):
  valid=bool(r['tracking'].get('valid')) and r['request']['request_source']=='tracking';t=r['local']['time_s'];yaw=float(a['yaw_world_rad'][i]);v=a['linear_velocity_world_m_s'][i,:2]
  if valid:
   ref=np.load(O/'local_plans'/f'frame_{i:03d}.npz');index=r['tracking']['projection_index']
   assert abs(float(ref['pose'][index,2])-r['tracking']['reference_body_heading_rad'])<1e-10
   assert abs(yaw-r['tracking']['root_heading_rad'])<1e-10
  rows.append(dict(frame=i,time_s=t,tracking_valid=valid,reference_version=r['local']['local_reference_version'],accepted_direction=r['diagnostics']['active_drive_direction'],source=r['request']['request_source'],reference_heading_rad=float(r['tracking']['reference_body_heading_rad']) if valid else None,actual_yaw_rad=yaw,actual_world_z_angular_velocity_rad_s=float(a['angular_velocity_world_rad_s'][i,2]),actual_body_forward_velocity_m_s=float(v[0]*np.cos(yaw)+v[1]*np.sin(yaw)),requested_curvature_1_m=r['tracking'].get('requested_curvature_1_m') if valid else None,lookahead_index=r['tracking'].get('lookahead_index') if valid else None,lookahead_accumulated_m=r['tracking'].get('lookahead_accumulated_m') if valid else None,raw_request=a['actions'][i].reshape(5,2)[0].tolist()))
 summaries=[]
 for phase in motion['actual_request_direction_phases']:
  i,j=phase['start_frame'],phase['end_frame'];ticks=fb[5*i:5*j];dt=np.diff(ph['simulation_time_s'][5*i:5*j+1]);unwrapped=np.unwrap(a['yaw_world_rad'][i:j+1]);valid=[row for row in rows[i:j] if row['tracking_valid']];heading=[abs(np.arctan2(np.sin(row['reference_heading_rad']-row['actual_yaw_rad']),np.cos(row['reference_heading_rad']-row['actual_yaw_rad']))) for row in valid]
  summary=dict(phase,actual_euler_yaw_delta_rad=float(unwrapped[-1]-unwrapped[0]),actual_world_z_integral_rad=float(np.dot(ph['angular_velocity_world_rad_s'][5*i+1:5*j+1,2],dt)),bank_error_rms_rad=float(np.sqrt(np.mean([r['error_rad']**2 for r in ticks]))),bank_error_peak_rad=max(abs(r['error_rad']) for r in ticks),pid_clipped_fraction=float(np.mean([r['pid_clipped'] for r in ticks])),supervisor_active_fraction=float(np.mean([r['supervisor_active'] for r in ticks])),valid_tracking_blocks=len(valid),heading_error_entry_rad=heading[0] if heading else None,heading_error_exit_rad=heading[-1] if heading else None,heading_error_peak_rad=max(heading) if heading else None)
  summaries.append(summary)
 result=dict(measured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,phase_heading_feedback=summaries,feedback_ticks=len(fb),offline_actual_descriptive_only=True,threshold_or_control_change=False,legacy_heading_diagnostic_not_used=True,bank_definition='base body Y world Z tilt asin(2(wx+yz)), normalized64 wxyz',yaw_is_euler_not_world_z_integral=True,whole_research_complete=False,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
 (H/'endpoint_navigation_heading_feedback_v2.json').write_text(json.dumps(result,indent=2)+'\n');print('SAVED ACTUAL HEADING FEEDBACK',len(rows),len(fb),len(summaries))
if __name__=='__main__':main()
