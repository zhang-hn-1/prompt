from types import SimpleNamespace
import numpy as np,pytest
from readonly_boundary_shadow_v1 import ReadOnlyBoundaryShadow

class Fake:
 def __init__(self):self.calls=0;self.tracker=SimpleNamespace(_reference_xy=np.array([[0.,0.],[1.,0.]]))
 def set_source_packet(self,p):self.source=p
 def set_current_packet(self,p):self.current=p
 def act(self,obs,goal):self.calls+=1;obs[0]=9;self.source['records'][0]['world'][0,0]=99;return np.zeros(10)
def packet(i):return dict(frame=i,time_s=.1*i,root_xyz=np.zeros(3),points_world=np.empty((0,3)))
def source():return dict(frame=0,time_s=0.,records=[dict(time_s=0.,world=np.zeros((1,3)))])
def test_reads_once_independent_state_does_not_mutate_candidate_inputs():
 b=Fake();s=ReadOnlyBoundaryShadow(b);obs=np.zeros(1);p=source();req=np.zeros(10);s.observe(req,obs,np.ones(2),p,packet(0));assert b.calls==1 and obs[0]==0 and p['records'][0]['world'][0,0]==0;np.testing.assert_array_equal(req,np.zeros(10))
def test_off_and_after_fork_do_not_read_payload():
 b=Fake();off=ReadOnlyBoundaryShadow(b,enabled=False);assert off.observe(None,None,None,None,None) is None and b.calls==0
 s=ReadOnlyBoundaryShadow(b);r=s.observe(np.ones(10),np.zeros(1),np.ones(2),source(),packet(0));assert r['first_request_fork'] and not s.active
 assert s.observe(None,None,None,None,None) is None and b.calls==1
def test_future_source_rejected_before_baseline_call():
 b=Fake();s=ReadOnlyBoundaryShadow(b);p=source();p['time_s']=1.
 with pytest.raises(ValueError):s.observe(np.zeros(10),np.zeros(1),np.zeros(2),p,packet(0))
 assert b.calls==0
