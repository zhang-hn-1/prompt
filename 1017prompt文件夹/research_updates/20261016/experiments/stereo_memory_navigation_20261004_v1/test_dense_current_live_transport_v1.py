import numpy as np
import pytest
from test_dense_current_source_packet_v1 import fixture

class CAD:
 def __init__(self):
  self.calls=0; self.frozen=dict(left=np.zeros((480,640),bool),right=np.zeros((480,640),bool))
 def project(self,q):
  self.calls+=1
  return self.frozen

def test_current_live_packet_dense_shadow_stays_sparse():
 from dense_current_live_transport_v1 import paired_sources
 prediction,pose,_,_=fixture();cad=CAD()
 shadow,current,masks=paired_sources(prediction,pose,np.zeros((5,17)),cad,.5)
 assert cad.calls==1
 assert shadow['query_pixel_id'].tolist()==[400*640+320]
 assert current['query_pixel_id'].tolist()==[400*640+320,401*640+321]
 assert np.array_equal(shadow['world'],current['world'][:1])
 assert current['source_model_admitted'] is False and current['certified_free'] is False

@pytest.mark.parametrize('bad_source,bad_pose',[(1.,.5),(.5,1.),(float('nan'),.5)])
def test_clock_rejected_before_cad_or_payload(bad_source,bad_pose):
 from dense_current_live_transport_v1 import paired_sources
 prediction,pose,_,_=fixture();prediction['time_s']=bad_source;pose['time_s']=bad_pose;cad=CAD()
 class Poison:
  def __array__(self,*a,**k):raise AssertionError('payload accessed before rejection')
 prediction['depth_m']=Poison()
 with pytest.raises(ValueError):paired_sources(prediction,pose,Poison(),cad,.5)
 assert cad.calls==0
