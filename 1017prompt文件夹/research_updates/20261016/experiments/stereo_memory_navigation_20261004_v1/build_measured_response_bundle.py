"""Archive measured whole responses; not an accepted arbitrary-state predictor."""
import json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
bundle={}
for name in ('positive','negative'):
    r=H/'large_bank_runs'/name;p=np.load(r/'physics.npz');i=np.flatnonzero(p['simulation_time_s']>=6-1e-5)[0]
    yaw=float(p['yaw_world_rad'][i]);c,s=np.cos(yaw),np.sin(yaw);R=np.array([[c,s],[-s,c]])
    bundle[name+'_relative_time_s']=p['simulation_time_s'][i:]-p['simulation_time_s'][i]
    bundle[name+'_measured_xy_in_initial_yaw_frame_m']=(p['position_world_xy'][i:]-p['position_world_xy'][i])@R.T
    # Prespecified requests, NOT source trajectory's future actual feedback actions.
    bundle[name+'_known_request_sequence']=np.load(r/'requests.npz')['requests'][60:]
    for key in ('root_quaternion_world_wxyz','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','smoothed_target','applied_action'):
        bundle[name+'_initial_'+key]=p[key][i]
    feedback=json.loads(next((r/'feedback').glob('bank_feedback_*.json')).read_text())
    (r/'initial_execution_state_at6s.json').write_text(json.dumps(dict(scope='current state before first nonzero block, lastPIDtick is5.98s not future',last_feedback_before_first_nonzero=feedback[299]),indent=2)+'\n')
np.savez_compressed(H/'measured_large_bank_whole_response_bundle.npz',**bundle)
