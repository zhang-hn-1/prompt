"""Explicitly offline reconstruction of mapping inputs from saved causal state/action pairs.
Native SimulationApp.close terminates workers before outer finally can write mapping records.
These records are reconstructed, NOT represented as original online telemetry.
"""
from pathlib import Path
import json,math,numpy as np
from bank_mapping import BankMap
h=Path(__file__).resolve().parent;model=BankMap(json.loads((h/'bank_mapping.json').read_text()));verification=[]
for p in sorted(h.glob('navigation_*/episode_*')):
 if p.name.endswith('.partial') or not (p/'metrics.json').exists():continue
 z=np.load(p/'states.npz');actions=np.load(p/'actions.npz')['action_blocks'].reshape(-1,5,2);rows=[];max_error=0.
 for i,controls in enumerate(actions):
  main=float(controls[0,0]);row=dict(block=i,active=False,origin='offline_saved_state_action_reconstruction',main_request_rad_s=main)
  if main!=0:
   control_speed=float(np.linalg.norm(z['linear_velocity_world_m_s'][i,:2]));speed=float(np.linalg.norm(np.asarray(z['linear_velocity_world_m_s'][i,:2],dtype=np.float64)));k=float(z['policy_desired_curvature_1_m'][i]);rate=float(z['angular_velocity_world_rad_s'][i,2]);choice=model.query(main,speed,k);sign=np.sign(main)
   correction=float(np.clip(-sign*.15*(k*sign*max(control_speed,.2)-rate),-math.radians(5),math.radians(5)))
   raw=choice['encoded_aux_rad']+correction;expected=float(np.clip(raw,-math.pi/6,math.pi/6));error=float(abs(expected-controls[0,1]));max_error=max(max_error,error);assert error<1e-6,(p.name,i,error)
   row.update(choice);row.update(active=True,new_aux_rad=raw,outer_yaw_correction_rad=correction,actual_protocol_aux_rad=float(controls[0,1]))
  else:assert np.all(controls==0)
  rows.append(row)
 out=p.parent/'feedback'/('mapping_reconstructed_'+p.name[len('episode_'):]+'.json');out.write_text(json.dumps(rows));verification.append(dict(episode=p.name,all_actual_requests_match=True,max_aux_error_rad=max_error,blocks=len(rows)))
(h/'mapping_reconstruction_verification.json').write_text(json.dumps(verification,indent=2));print('Reconstructed and verified episodes:',len(verification))
