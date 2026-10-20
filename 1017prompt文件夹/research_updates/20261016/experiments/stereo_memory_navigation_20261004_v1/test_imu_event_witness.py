import pytest
from imu_event_witness import assess
def rows():return [dict(begin_s=i*.02,end_s=(i+1)*.02,sensor_time_s=i*.02,valid=True,native_z_rad_s=.3 if i%2 else -.4,current_rate_rad_s=.3 if i%2 else -.4,current_body_error_rad_s=0.) for i in range(50)]
def test_begin_current_identified():
 a=assess(rows());assert a['begin_current_pass'] and not a['end_current_pass'];assert a['evaluation_rows']==48
@pytest.mark.parametrize('mode',['end','old','invalid','missing'])
def test_different_hypothesis_not_misidentified(mode):
 r=rows()
 if mode=='missing':r.pop()
 if mode=='invalid':r[20]['valid']=False
 if mode=='end':
  for q in r:q['sensor_time_s']=q['end_s']
 if mode=='old':
  for i in range(1,len(r)):r[i]['native_z_rad_s']=r[i-1]['current_rate_rad_s']
 a=assess(r);assert not a['begin_current_pass']
 if mode=='end':assert a['end_current_pass']
