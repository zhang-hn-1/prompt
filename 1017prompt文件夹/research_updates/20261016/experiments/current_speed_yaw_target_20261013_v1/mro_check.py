from pathlib import Path
import sys,json,math,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');H=W/'experiments/stereo_memory_navigation_20261004_v1'
sys.path[:0]=[str(D),str(H),str(P),str(W/'experiments/visible_surface_connectivity_20261004_v1')]
from scripts.run_pointgoal_benchmark import load_action_protocol
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT
from outgoing_lookahead import make_endpoint_oracle
from cusp_coast_acquisition_v1 import with_cusp_coast_tracker
from speed_target import with_current_speed_target,feedback
p=load_action_protocol(DEFAULT_CONTRACT.resolve());checks=[]
for main in [-1.,1.]:
 for speed in [0.,.1,.2,.4]:
  a=make_endpoint_oracle(p);a.__class__=with_cusp_coast_tracker(type(a));b=make_endpoint_oracle(p);b.__class__=with_current_speed_target(with_cusp_coast_tracker(type(b)))
  for obj in [a,b]:
   obj.mapping_records=[];obj._tracking_records=[];obj._desired_curvature_1_m=.7
   _,old=feedback(main,max(speed,.2),.7,.01,obj.yaw_rate_feedback_gain_s)
   obj._tracking_audit=dict(valid=True,world_xy_speed_m_s=speed,measured_world_yaw_rate_rad_s=.01,desired_world_yaw_rate_rad_s=.7*np.sign(main)*max(speed,.2),yaw_rate_feedback_auxiliary_rad=old)
  x=a._request_action(main,.1+old,source='tracking');y=b._request_action(main,.1+old,source='tracking')
  _,new=feedback(main,speed,.7,.01,b.yaw_rate_feedback_gain_s)
  rec=b.mapping_records[-1];assert np.isclose(rec['new_aux_rad'],rec['encoded_aux_rad']+new);assert x[0]==y[0]
  if speed>=.2:assert np.array_equal(x,y)
  assert np.isclose(b._tracking_audit['policy_raw_auxiliary_target_rad'],rec['encoded_aux_rad']+new)
  checks.append(dict(main=main,speed=speed,baseline=x.tolist(),candidate=y.tolist(),single_correction=True))
(D/'mro_checks.json').write_text(json.dumps(dict(count=len(checks),checks=checks,MRO=[c.__name__ for c in type(b).__mro__]),indent=2)+'\n');print('full tracker dispatch checks',len(checks))
