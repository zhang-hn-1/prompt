from pathlib import Path
import hashlib,json,numpy as np
H=Path(__file__).resolve().parent;O=H/'oracle_rgb_shadow_run';old=H.parent/'oracle_feasible_route_20260929_v9/navigation_hybrid_golden_golden_e1_0001/episode_golden_e1_0001';a=np.load(O/'episode_arrays.npz');b=np.load(old/'states.npz');c=np.load(old/'actions.npz')
np.testing.assert_equal(a['actions'],c['action_blocks']);np.testing.assert_equal(a['positions_world_xy'],b['position_world_xy'])
fields={}
for k in ['yaw_world_rad','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','simulation_time_s','physics_step_index','raw_simulation_time_s','raw_physics_step_index','roll_world_rad']:
 if k in a.files and k in b.files:
  np.testing.assert_equal(a[k],b[k]);fields[k]=dict(shape=list(a[k].shape),exact=True)
result=dict(actions_exact=True,actions=len(a['actions']),position_exact=True,states=len(a['positions_world_xy']),telemetry=fields,scope='Exact saved historical action/physics telemetry parity despite current rig/lighting/capture/runner recording differences. Old environment did not identify its rig. No new simulation/on-off control A/B, no same-RGB claim.',old_state_sha256=hashlib.sha256((old/'states.npz').read_bytes()).hexdigest(),old_action_sha256=hashlib.sha256((old/'actions.npz').read_bytes()).hexdigest())
(H/'oracle_rgb_shadow_historical_parity.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
