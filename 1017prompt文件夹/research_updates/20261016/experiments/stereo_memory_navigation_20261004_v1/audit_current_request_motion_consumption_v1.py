"""Postclosed current request/reference/actual velocity consumption; no prediction or fit."""
from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent

def stats(a):
 a=np.asarray(a,float)
 return dict(count=len(a),min=float(a.min()),median=float(np.median(a)),max=float(a.max()),mean=float(a.mean())) if len(a) else dict(count=0)

def main():
 out={};inputs={}
 for arm in ('b0','candidate'):
  d=H/f'bounded_prefix_e1_{arm}_run_v1';rf=d/'policy_records.json';sf=d/'states.json';rows=json.loads(rf.read_text());states=json.loads(sf.read_text());groups={}
  for r in rows:
   c=r.get('current_tracking_chain',{});s=r['tracker_state_after_act']
   if not c.get('fresh_tracker_record') or not c.get('valid') or c.get('source')!='tracking' or s['pending_direction'] is not None or s['braking_state']!='tracking':continue
   direction=c['reference_drive_direction']
   if direction!=s['active_direction']:continue
   v=np.asarray(states[r['frame']]['velocity'][:2]);np.testing.assert_array_equal(v.astype(np.float32),np.array([c['velocity_world_x_m_s'],c['velocity_world_y_m_s']],np.float32));theta=c['reference_motion_tangent_rad'];tangent=np.array([np.cos(theta),np.sin(theta)]);q=np.asarray(r['request']);w=r['current_reference_waypoints_after_act'][c['projection_index']]
   groups.setdefault(direction,[]).append(dict(frame=r['frame'],nominal_carrier_main=w['main_command_rad_s'],issued_main=float(q[0]),nominal_aux=w['auxiliary_angle_rad'],issued_aux=float(q[1]),actual_speed_m_s=float(np.linalg.norm(v)),actual_current_tangent_speed_m_s=float(v@tangent)))
  out[arm]={k:dict(samples=rs,nominal_carrier_main=stats([r['nominal_carrier_main'] for r in rs]),issued_main=stats([r['issued_main'] for r in rs]),actual_speed=stats([r['actual_speed_m_s'] for r in rs]),actual_current_reference_tangent_speed=stats([r['actual_current_tangent_speed_m_s'] for r in rs]),nonpositive_tangent_samples=sum(r['actual_current_tangent_speed_m_s']<=0 for r in rs)) for k,rs in groups.items()}
  for p in (rf,sf):inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
 p=H/'current_request_motion_consumption_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(dict(arms=out,input_sha256=inputs,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),scope='Only current freshly valid normal-tracking rows. Installed nominal carrier and final issued request are different layers; current measured XY speed is descriptive, not a forward prediction or geometric curvature. Own reference tangent differs between branches after120; statistics are not a paired instantaneous causal estimate.',no_fit_or_solver_or_physics=True,no_domain_admission=True,whole_mission_complete=False),indent=2)+'\n');print(json.dumps({a:{k:{key:val for key,val in v.items() if key!='samples'} for k,v in rs.items()} for a,rs in out.items()}))
if __name__=='__main__':main()
