import numpy as np
from assess_qualified_start_navigation_e1_v1 import acquisition_speed
from standby_clock_eligibility_v1 import ClockQualifiedStandby

def test_boundary_uses_actual_float32_norm_and_resets_count():
 velocity=np.array([.03,.04,0.],np.float32);actual=float(np.linalg.norm(velocity[:2]));s=acquisition_speed(velocity)
 assert s==actual and s>=.05
 assert float(np.linalg.norm(velocity.astype(float)[:2]))<.05
 c=ClockQualifiedStandby(enabled=True)
 for i in range(4):c.choose(i*.1,[0,0],0.,dict(frame=i,time_s=i*.1,records=[]))
 assert c.choose(.4,[0,0],s,dict(frame=4,time_s=.4,records=[])) is not None and c.standby_low_count==0
