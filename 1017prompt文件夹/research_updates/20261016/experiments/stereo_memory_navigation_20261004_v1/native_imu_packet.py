"""Readonly native sensor boundary. Qualification comparison is not model input."""
import numpy as np
def native_packet(reading,time_s,raw_time_s,encoder,expected_gyro):
 valid=bool(reading.is_valid);st=float(reading.time)
 g=np.array([reading.ang_vel_x,reading.ang_vel_y,reading.ang_vel_z],float)
 a=np.array([reading.lin_acc_x,reading.lin_acc_y,reading.lin_acc_z],float)
 t=float(time_s);raw=float(raw_time_s);enc=float(encoder);expected=np.asarray(expected_gyro,float)
 finite=np.isfinite([st,t,raw,enc]).all() and np.isfinite(g).all() and np.isfinite(a).all() and expected.shape==(3,) and np.isfinite(expected).all()
 age=raw-st if finite else None;error=float(np.linalg.norm(g-expected)) if finite else None
 reason='invalid' if not valid else ('nonfinite' if not finite else ('time_mismatch' if abs(age)>1e-5 else ('gyro_axes_mismatch' if error>1e-5 else None)))
 d=dict(time_s=t,raw_time_s=raw,sensor_time_s=st,is_valid=valid,gyro_body_rad_s=g.tolist(),linear_acceleration_body_m_s2=a.tolist(),age_s=age,gyro_comparison_error_rad_s=error,qualified=reason is None,reason=reason,orientation_read=False)
 if reason:return None,d
 return dict(time_s=t,gyro_body_rad_s=g.copy(),linear_acceleration_body_m_s2=a.copy(),main_encoder_rad_s=enc),d
