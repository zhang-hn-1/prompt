import numpy as np
from assess_forward_outer_yaw_e1_v1 import intervention_evidence

def test_no_override_or_earlier_nonconsumer_fork_cannot_pass():
 b=np.tile([.9,.13],(3,5)).astype(np.float32);c=b.copy();c[0,1]=.14;rows=[dict(frame=i,forward_outer_yaw_override=False,request=c[i].tolist()) for i in range(3)]
 r=intervention_evidence(0,b,c,rows);assert r['count']==0 and not r['consumer_fork_isolated']
 c[2,::2]=.5;rows[2].update(forward_outer_yaw_override=True,forward_outer_yaw_parent_request=b[2].tolist(),request=c[2].tolist());r=intervention_evidence(0,b,c,rows);assert r['count']==1 and r['first_override']==2 and not r['consumer_fork_isolated']

def test_actual_same_parent_override_and_aux_at_fork_required():
 b=np.tile([.9,.13],(3,5)).astype(np.float32);c=b.copy();c[1,::2]=.5;rows=[dict(frame=i,forward_outer_yaw_override=i==1,forward_outer_yaw_parent_request=b[i].tolist(),request=c[i].tolist()) for i in range(3)]
 r=intervention_evidence(1,b,c,rows);assert r['count']==1 and r['consumer_fork_isolated']
 rows[1]['forward_outer_yaw_parent_request'][1]=.2
 assert not intervention_evidence(1,b,c,rows)['consumer_fork_isolated']
