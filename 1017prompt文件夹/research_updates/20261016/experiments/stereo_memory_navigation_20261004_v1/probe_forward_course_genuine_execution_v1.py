"""Synthetic current-state CPU consumption; not a physical navigation label."""
from pathlib import Path
import json,hashlib,sys
import numpy as np
from test_forward_course_geometry_real_factory_v3 import make,current,observation
from scripts.action_smoothing import ActionSmoother
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import BankPID
results={}
for enabled in (False,True):
 p=make(enabled);o=observation();actions=[]
 for f in range(70,75):current(p,f,.1*f);actions.append(p.act(o,np.array([3.,0.])).copy())
 q=actions[-1].reshape(5,2);pid=BankPID(kp=.4,ki=.1,kd=.7,dt=.02);smoother=ActionSmoother(dt_s=.02,max_main_acceleration_rad_s2=.5,max_auxiliary_rate_rad_s=np.pi/6,max_auxiliary_velocity_rad_s=.3);sm=[];corrected=[]
 for target in q:
  c=pid.step(target,0.,downstream_blocked=False);corrected.append(c.tolist())
  sm.append(smoother.step(c,[-2.,-np.pi/6],[2.,np.pi/6],auxiliary_position_rad=0.,auxiliary_velocity_rad_s=0.).tolist())
 results['candidate' if enabled else 'b0']=dict(issued_request=actions[-1].tolist(),current_tracking_record=p.records[-1]['current_tracking_chain'],geometry_decision=p.records[-1]['forward_course_geometry'],PID_corrected=corrected,CPU_applied=sm)
assert results['b0']['issued_request']!=results['candidate']['issued_request']
assert results['b0']['CPU_applied']!=results['candidate']['CPU_applied']
paths=[Path(__file__).resolve(),H/'forward_course_geometry_adapter_v3.py',H/'forward_course_geometry_v1.py',H/'test_forward_course_geometry_real_factory_v3.py']
out=H/'forward_course_genuine_execution_probe_v1.json';assert not out.exists();out.write_text(json.dumps(dict(arms=results,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},synthetic_known_current_state=True,no_physics_run=True,feedback_each_tick_fixed_zero_bank_joint=True,not_navigation_or_dynamics_prediction=True),indent=2)+'\n')
print(json.dumps({k:{x:v[x] for x in ['issued_request','CPU_applied']} for k,v in results.items()}))
