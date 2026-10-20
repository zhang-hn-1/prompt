import pytest
from local_endpoint_event_v2 import observe_local_endpoint

def test_future_clock_is_rejected_before_any_geometry_payload_read():
 class PayloadTrap:
  def __getitem__(self,k):
   if k=='time_s':return 2.
   if k=='feedback_time_s':return 2.1
   raise AssertionError('geometry read before rejecting future clock')
 with pytest.raises(ValueError,match='clock'):
  observe_local_endpoint(PayloadTrap(),lambda *a:0.)
