"""Frozen physical labels; never online prediction or angle-target substitution."""
import numpy as np
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence
from bank_response_measurement import summarize,_measurement
from straight_pacing_measurement_v1 import stop_qualification

def measured_arm(status,identity,states,contacts,physics,n):
 result=dict(classification='unassessed_execution',complete=False)
 if status.get('status')!='finished' or status.get('child_returncode')!=0:return result
 result['classification']='unassessed_evidence'
 try:
  if not 0<=n<=300 or identity.get('actions')!=n or identity.get('captures')!=n+1:raise ValueError('identity coverage')
  normalized=complete_execution_evidence(states,contacts,physics['simulation_time_s'],n)
 except (KeyError,ValueError,TypeError):return result
 if any(x['collision'] for x in normalized):
  result.update(classification='measured_collision_reject',first_contact_s=min(x['time_s'] for x in normalized if x['collision']));return result
 if n!=300 or identity.get('complete') is not True:return result
 t=np.array(physics['simulation_time_s']);v=np.array(physics['linear_velocity_world_m_s']);mask=(t>=19)&(t<=21.5)
 w=summarize(t,physics['position_world_xy'],physics['root_quaternion_world_wxyz'],v[:,:2],physics['yaw_world_rad'],1.,19.,21.5)
 result.update(classification='complete_response',complete=True,window=w,actual_mean_speed_m_s=float(np.mean(np.linalg.norm(v[mask,:2],axis=1))),actual_mean_bank_deg=float(np.mean(np.degrees(_measurement.bank(physics['root_quaternion_world_wxyz']))[mask])),terminal_stop=stop_qualification([s['time_s'] for s in states],[s['velocity'] for s in states],22.))
 return result
