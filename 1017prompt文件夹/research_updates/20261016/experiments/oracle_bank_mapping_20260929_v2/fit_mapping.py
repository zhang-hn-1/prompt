from pathlib import Path
import json,math,numpy as np
from measurement import geometry,bank
from bank_mapping import BankMap
h=Path(__file__).resolve().parent;rows=[]
for p in sorted(h.glob('calibration_*/physics.npz')):
 cfg=json.loads((p.parent/'config.json').read_text());main=cfg['main'];sgn=np.sign(main);z=np.load(p);t=z['simulation_time_s'];v,k=geometry(z['position_world_xy']);b=np.degrees(bank(z['root_quaternion_world_wxyz']));velocity=z['linear_velocity_world_m_s'][:,:2];heading=z['yaw_world_rad'];signed=velocity[:,0]*np.cos(heading)+velocity[:,1]*np.sin(heading)
 for end,target in [(25,15),(45,22.5),(65,-15),(85,-22.5),(100,0)]:
  mask=(t>=end-5)&(t<=end-.5);valid=mask&np.isfinite(k);values=k[valid]*sgn;med=float(np.median(values));iqr=float(np.ptp(np.percentile(values,[25,75])));span=float(np.ptp(np.percentile(b[valid],[5,95])));direction_fraction=float(np.mean(signed[valid]*sgn>0));ratio=float(valid.sum()/mask.sum())
  sensitivity={}
  for window in [21,41]:
   _,kk=geometry(z['position_world_xy'],window=window);sensitivity[str(window)]=float(np.nanmedian(kk[mask])*sgn)
  checks=dict(valid_fraction=ratio>=.9,curvature_iqr=iqr<=max(.2,.3*abs(med)),bank_span=span<=3,direction=direction_fraction>=.9)
  rows.append(dict(case=p.parent.name,main=float(main),requested_bank_deg=target,window_start_s=end-5,window_end_s=end-.5,actual_speed_m_s=float(np.median(v[valid])),actual_bank_deg=float(np.median(b[valid])),curvature_body_1_m=med,curvature_geometric_1_m=med*sgn,curvature_iqr=iqr,bank_span_95_5_deg=span,valid_fraction=ratio,direction_match_fraction=direction_fraction,sensitivity=sensitivity,checks=checks,accepted=all(checks.values())))
model={'schema':'measured_bank_inverse_v1','directions':{},'rejected':[],'measurement':'steady tail windows, closed-loop frozen PID, requested bank -> body curvature'}
for direction,sgn in [('forward',1),('reverse',-1)]:
 levels={}
 for label,mag in [('low',.5),('high',1.5)]:
  rr=sorted([r for r in rows if r['main']==sgn*mag and r['accepted']],key=lambda x:x['requested_bank_deg'])
  if len(rr)!=5:model['rejected'].append(dict(direction=direction,level=label,reason='not all five predefined windows accepted',accepted=len(rr)));continue
  kk=[r['curvature_body_1_m'] for r in rr]
  if not all(np.diff(kk)<0):model['rejected'].append(dict(direction=direction,level=label,reason='nonmonotone curvature',values=kk));continue
  levels[label]=dict(speed_m_s=float(np.median([r['actual_speed_m_s'] for r in rr])),bank_rad=[math.radians(r['requested_bank_deg']) for r in rr],curvature_body_1_m=kk,actual_bank_deg=[r['actual_bank_deg'] for r in rr])
 if len(levels)==2:model['directions'][direction]=levels
BankMap(model)
(h/'calibration_assessment.json').write_text(json.dumps(rows,indent=2));(h/'bank_mapping.json').write_text(json.dumps(model,indent=2))
for r in rows:print(r['main'],r['requested_bank_deg'],'speed',round(r['actual_speed_m_s'],3),'bank',round(r['actual_bank_deg'],2),'k',round(r['curvature_body_1_m'],3),'IQR',round(r['curvature_iqr'],3),'accepted',r['accepted'])
print('supported:',list(model['directions']),'rejected:',model['rejected'])
