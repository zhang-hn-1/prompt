from pathlib import Path
import sys,json,math,numpy as np
h=Path(__file__).resolve().parent;sys.path.insert(0,str(h.parent/'oracle_pid_validation_20260929_v4'))
from measurement import geometry
old=h.parent/'oracle_pid_validation_20260929_v4/navigation_hard_0';output=[]
def kappa(p,yaw,target):
 d=target-p;return 2*(-np.sin(yaw)*d[0]+np.cos(yaw)*d[1])/max(float(d@d),1e-9)
for case in ['0002','0004']:
 eid=f'hard_0_e1_{case}';z=np.load(old/f'episode_{eid}/states.npz');ref=z['kinodynamic_reference_pose'];dirs=z['kinodynamic_reference_drive_direction'];mp=json.loads((old/'feedback'/f'mapping_{eid}.json').read_text());fb=json.loads((old/'feedback'/f'bank_feedback_{eid}.json').read_text());rows=[];_,actual=geometry(z['position_world_xy'],dt=.1,window=7)
 for i,r in enumerate(mp):
  if not r['active']:rows.append(dict(block=i,time=float(z['simulation_time_s'][i]),source=r['source'],valid=False));continue
  j=int(z['policy_reference_index'][i]);speed=float(np.linalg.norm(z['linear_velocity_world_m_s'][i,:2]));look=max(.5,float(np.clip(.45+.8*speed,.45,.9)));target=j;length=0.
  while target+1<len(ref):
   if dirs[target+1]!=dirs[j]:break
   length+=float(np.linalg.norm(ref[target+1,:2]-ref[target,:2]));target+=1
   if length>=look:break
  xy=z['position_world_xy'][i];yaw=float(z['yaw_world_rad'][i]);ideal=kappa(ref[j,:2],ref[j,2],ref[target,:2]);pos=kappa(xy,ref[j,2],ref[target,:2]);request=kappa(xy,yaw,ref[target,:2]);assert abs(request-r['requested_curvature_body_1_m'])<1e-6,(i,request,r)
  samples=fb[5*i:5*i+5];head=float(np.arctan2(np.sin(ref[j,2]-yaw),np.cos(ref[j,2]-yaw)));signed=float(np.dot(z['linear_velocity_world_m_s'][i,:2],[np.cos(yaw),np.sin(yaw)]));rows.append(dict(block=i,time=float(z['simulation_time_s'][i]),source=r['source'],valid=True,index=j,target_index=target,main=r['main_request_rad_s'],ideal_geometry_curvature=ideal,position_increment=pos-ideal,heading_increment=request-pos,requested_curvature=request,heading_error_deg=float(np.degrees(head)),curvature_clamped=r['curvature_clamped'],aux_mapping_deg=float(np.degrees(r['encoded_aux_rad'])),outer_yaw_correction_deg=float(np.degrees(r['outer_yaw_correction_rad'])),desired_bank_deg=float(np.degrees(samples[0]['desired_bank_rad'])),bank_deg=float(np.degrees(samples[0]['bank_rad'])),pid_aux_deg=float(np.degrees(samples[0]['pid_protocol_aux_rad'])),applied_aux_deg=float(np.degrees(samples[0]['applied_aux_rad'])),actual_aux_deg=float(np.degrees(z['joint_position_rad'][i,1])),actual_body_curvature=float(actual[i]*np.sign(signed)) if np.isfinite(actual[i]) else None,speed=speed))
 (h/f'chain_{case}.json').write_text(json.dumps(rows,indent=2));valid=[r for r in rows if r['valid']];last=[r for r in valid if r['time']>=rows[-1]['time']-4];print(case,'LAST4',json.dumps(last[::10],indent=2));output.append(dict(case=case,tracking_blocks=len(valid),clamped_fraction=float(np.mean([r['curvature_clamped'] for r in valid])),last4=dict(n=len(last),mean_abs_ideal=float(np.mean([abs(r['ideal_geometry_curvature']) for r in last])),mean_abs_position_increment=float(np.mean([abs(r['position_increment']) for r in last])),mean_abs_heading_increment=float(np.mean([abs(r['heading_increment']) for r in last])),mean_abs_request=float(np.mean([abs(r['requested_curvature']) for r in last])),mean_abs_heading_error_deg=float(np.mean([abs(r['heading_error_deg']) for r in last])))))
(h/'chain_summary.json').write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2))
