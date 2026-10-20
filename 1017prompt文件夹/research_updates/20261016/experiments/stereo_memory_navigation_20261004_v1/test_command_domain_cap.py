import numpy as np
import pytest
from command_domain_cap import with_command_domain_cap
class Base:
 def __init__(self,nominal,unknown=1):self.nominal=np.asarray(nominal,np.float32);self.unknown=unknown;self.calls=0;self.records=[]
 def act(self,obs,goal):
  self.calls+=1;out=self.nominal.copy()
  if self.unknown>0:out[::2]=np.clip(out[::2],-.3,.3)
  self.records.append(dict(unknown_fraction=self.unknown,uncapped_request=self.nominal.tolist(),request=out.tolist()));return out

def test_single_call_effective_main_request_only():
 p=with_command_domain_cap(Base)([1,.24]*5);out=p.act(None,None);assert p.calls==1;np.testing.assert_equal(out[::2],np.full(5,.5,np.float32));np.testing.assert_equal(out[1::2],p.nominal[1::2]);assert p.records[-1]['tracking_fields_scope']=='tracker_before_task_cap';assert p.records[-1]['unknown_main_cap_rad_s']==.5

def test_settling_no_route_zero_stays_zero():
 p=with_command_domain_cap(Base)(np.zeros(10));np.testing.assert_equal(p.act(None,None),np.zeros(10));assert p.calls==1

def test_no_unknown_does_not_change_controller_request():
 p=with_command_domain_cap(Base)([1,.24]*5,unknown=0);np.testing.assert_equal(p.act(None,None),p.nominal);assert p.calls==1

def test_nonfinite_uncapped_request_is_not_executed():
 p=with_command_domain_cap(Base)([np.nan,.24]*5)
 with pytest.raises(ValueError):p.act(None,None)

def test_reverse_request_domain_stays_at_old_cap():
 p=with_command_domain_cap(Base)([-1,.24]*5);np.testing.assert_equal(p.act(None,None)[::2],np.full(5,-.3,np.float32))
