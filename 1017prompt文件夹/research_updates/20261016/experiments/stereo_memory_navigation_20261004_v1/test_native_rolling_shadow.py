import numpy as np,pytest
from native_rolling_shadow import NativeRollingShadow
from imu_gravity_rolling import GravityRolling
def packet(t=0):return dict(time_s=t,gyro_body_rad_s=[.1,.2,.3],linear_acceleration_body_m_s2=[0,0,9.81],main_encoder_rad_s=.5)
def test_frozen_physics_response_and_copied_whitelist():
 shadow=NativeRollingShadow();model=GravityRolling([0,0,1]);rows=[]
 for t in [0,.02,.04]:
  p=packet(t);expected=model.update(p);assert shadow.consume(p,[0,0,1] if t==0 else None) is None
  for k in ['position','rotation','velocity']:np.testing.assert_array_equal(shadow.records[-1][k],expected[k])
  rows.append(p)
 rows[-1]['gyro_body_rad_s'][0]=999
 assert shadow.sensors[-1]['gyro_body_rad_s'][0]==.1
 with pytest.raises(ValueError):shadow.consume(dict(packet(.06),root_quaternion=[1,0,0,0]))
def test_initial_calibration_and_future_prefix():
 with pytest.raises(ValueError):NativeRollingShadow().consume(packet())
 full=NativeRollingShadow();prefix=NativeRollingShadow()
 for i in range(5):
  full.consume(packet(i*.02),[0,0,1] if i==0 else None)
  if i<3:prefix.consume(packet(i*.02),[0,0,1] if i==0 else None)
 assert full.records[:3]==prefix.records
