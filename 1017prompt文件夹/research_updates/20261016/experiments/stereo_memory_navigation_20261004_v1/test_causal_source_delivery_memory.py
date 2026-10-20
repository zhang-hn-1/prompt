import numpy as np,pytest
from causal_source_delivery_memory import CausalSourceDeliveryMemory
def packet(frame=1,t=1.):return dict(frame=frame,time_s=t,world=np.array([[.2,.1,.4]]),query_pixel_id=np.array([1]),partner_pixel_id=np.array([2]),input_sha256={'source':'a'*64})
class Poison:
 def __getitem__(self,key):raise AssertionError('future payload/header read')
def test_future_availability_rejected_before_any_packet_read():
 m=CausalSourceDeliveryMemory()
 with pytest.raises(ValueError):m.deliver(Poison(),available_time_s=3.,now=2.)
 assert m.snapshot(2.)['records']==[]
def test_late_delivery_preserves_birth_provenance_and_expiration():
 m=CausalSourceDeliveryMemory();p=packet();m.deliver(p,available_time_s=2.,now=2.);p['world'][:]=999;s=m.snapshot(3.);assert s['source_time_s']==1 and s['records'][0]['time_s']==1 and s['records'][0]['available_time_s']==2;np.testing.assert_array_equal(s['records'][0]['world'],[[.2,.1,.4]])
 assert len(m.snapshot(11.)['records'])==1
 assert m.snapshot(11.00001)['records']==[]
def test_available_before_acquisition_and_duplicate_are_atomic_errors():
 m=CausalSourceDeliveryMemory()
 with pytest.raises(ValueError):m.deliver(packet(),available_time_s=.5,now=2.)
 assert m.snapshot(2.)['records']==[]
 m.deliver(packet(),available_time_s=2.,now=2.)
 with pytest.raises(ValueError):m.deliver(packet(),available_time_s=3.,now=3.)
 assert m.snapshot(3.)['source_time_s']==1 and m.snapshot(3.)['available_time_s']==2
