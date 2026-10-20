from pathlib import Path
import math,json,numpy as np
from speed_target import feedback,with_current_speed_target
D=Path(__file__).resolve().parent;checks=[]
for main in [-1,1]:
 for speed in [.2,.4]:
  desired,cor=feedback(main,speed,.7,.01,.15);sign=np.sign(main);old=float(np.clip(-sign*.15*(.7*sign*max(speed,.2)-.01),-math.radians(5),math.radians(5)));assert cor==old;checks.append('in_domain_exact')
 desired,cor=feedback(main,0,.7,0,.15);assert desired==cor==0;checks.append('zero_target')
for args in [(1,-1,0,0,.15),(0,1,0,0,.15),(1,.1,np.nan,0,.15)]:
 try:feedback(*args)
 except ValueError:checks.append('invalid_reject')
 else:raise AssertionError
class Base:
 def _request_action(self,m,a,*,source):return np.array([m,a])
X=with_current_speed_target(Base)();X.yaw_rate_feedback_gain_s=.15;X._desired_curvature_1_m=.7;X._tracking_audit=dict(valid=True,world_xy_speed_m_s=.1,measured_world_yaw_rate_rad_s=.01,yaw_rate_feedback_auxiliary_rad=-.0195,desired_world_yaw_rate_rad_s=.14);q=X._request_action(1,.1,source='tracking');assert q[0]==1 and abs(q[1]-.1105)<1e-8;checks.append('aux_only_wrapper');X._request_action(0,0,source='braking');assert X.current_speed_last is None;checks.append('nontracking_unchanged')
(D/'interface_checks.json').write_text(json.dumps(dict(checks=checks,count=len(checks),scope='synthetic scalar/wrapper plus exactexisting source_ast; not navigation efficacy'),indent=2)+'\n');print(len(checks),'checks')
