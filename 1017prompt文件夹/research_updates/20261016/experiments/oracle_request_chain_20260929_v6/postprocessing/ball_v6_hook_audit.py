from pathlib import Path
import sys,json,math,numpy as np
h=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_request_chain_20260929_v6');sys.path.insert(0,str(h));from course_geometry import curvature_from_motion
out=[]
for c in ['0002','0004']:
 run=h/f'navigation_course_{c}';eid=f'hard_0_e1_{c}';z=np.load(run/f'episode_{eid}/states.npz');track=json.loads((run/'feedback'/f'tracking_{eid}.json').read_text());mapping=json.loads((run/'feedback'/f'mapping_{eid}.json').read_text());ref=z['kinodynamic_reference_pose'];errs=[];gaps=[];count=0
 for i,r in enumerate(track):
  if not r.get('valid') or r['source']!='tracking':continue
  target=ref[r['lookahead_index'],:2];assert np.array_equal(target,np.array([r['lookahead_x_m'],r['lookahead_y_m']]))
  k,active=curvature_from_motion(target-z['position_world_xy'][i],z['linear_velocity_world_m_s'][i,:2],z['yaw_world_rad'][i],r['reference_drive_direction'],r['geometric_pure_pursuit_curvature_1_m']);assert active==r['candidate_active'];errs.append(abs(k-r['requested_curvature_1_m']));assert abs(k-mapping[i]['requested_curvature_body_1_m'])<1e-6;count+=1
  if active:
   speed=np.linalg.norm(z['linear_velocity_world_m_s'][i,:2]);course=np.arctan2(*z['linear_velocity_world_m_s'][i,:2][::-1]);gaps.append(float(abs(np.arctan2(np.sin(course-z['yaw_world_rad'][i]),np.cos(course-z['yaw_world_rad'][i])))))
 assert max(errs)<1e-6
 ids=np.flatnonzero(z['physical_contact']);out.append(dict(case=c,validated_tracking_blocks=count,max_hook_curvature_error=max(errs),same_target_verified=True,first_contact_time_s=float(z['simulation_time_s'][ids[0]]) if len(ids) else None,last_contact_time_s=float(z['simulation_time_s'][ids[-1]]) if len(ids) else None,contact_samples=len(ids),formal_clearance_at_first_contact_m=float(z['clearance_m'][ids[0]]) if len(ids) else None,course_body_heading_abs_max_deg=float(np.degrees(max(gaps)))))
(h/'online_hook_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
