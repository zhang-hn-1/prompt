"""Native begin-label interval contract; actual step clocks, no truth reindexing."""
import numpy as np
def interval_packet(reading,begin_raw_s,end_raw_s,relative_end_s,encoder):
 t=float(relative_end_s);begin=float(begin_raw_s);end=float(end_raw_s);stamp=float(reading.time);enc=float(encoder);valid=bool(reading.is_valid)
 g=np.array([reading.ang_vel_x,reading.ang_vel_y,reading.ang_vel_z],float);a=np.array([reading.lin_acc_x,reading.lin_acc_y,reading.lin_acc_z],float)
 finite=np.isfinite([t,begin,end,stamp,enc]).all() and np.isfinite(g).all() and np.isfinite(a).all()
 reason='invalid' if not valid else ('nonfinite' if not finite else ('wrong_interval' if abs(end-begin-.02)>1e-6 else ('stamp_outside_begin' if abs(stamp-begin)>1e-5 else None)))
 d=dict(time_s=t,begin_raw_s=begin,end_raw_s=end,sensor_begin_stamp_s=stamp,qualified=reason is None,reason=reason,is_valid=valid,gyro_body_rad_s=g.tolist(),linear_acceleration_body_m_s2=a.tolist(),body_basis='native body axes at interval begin',orientation_read=False,source_contract_version='actual_interval_v2')
 if reason:return None,d
 return dict(time_s=t,gyro_body_rad_s=g.copy(),linear_acceleration_body_m_s2=a.copy(),main_encoder_rad_s=enc),d
