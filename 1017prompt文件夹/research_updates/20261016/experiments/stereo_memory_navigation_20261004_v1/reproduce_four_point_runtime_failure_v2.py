from pathlib import Path
import sys,json,numpy as np,traceback,hashlib,datetime
from benchmark.policies import PolicyObservation
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from supported_four_point_consumer import with_supported_four_points
from prior_rgb_navigation import PriorRGBPolicy
from command_domain_cap import with_command_domain_cap
from persistent_core_grid import PersistentCoreGrid
from observation_clock_adapter import observe_clock_without_act
from estimated_navigation_observation import EstimatedNavigationObservation
import scripts.run_pointgoal_benchmark as runner
H=Path(__file__).resolve().parent;D=H/'supported_four_navigation_R1_run_v1'
def reconstruct():
 protocol=runner.load_action_protocol(P/'configs/data_contract.yaml');p=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current');init=np.load(D/'estimated_poses/frame_000.npz');p.grid=PersistentCoreGrid(init['root_position_world_m'][:2]-20,60)
 for i in range(140):
  pose=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');packet=dict(frame=i,time_s=float(pose['time_s']),points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m']);observe_clock_without_act(p,packet,float(pose['time_s']))
 pose=dict(np.load(D/'estimated_poses/frame_140.npz'));history=np.load(D/'history/frame_140.npz');images=np.load(D/'online_inputs/frame_140.npz');src=dict(np.load(D/'sensor_packets/source_140.npz'));seed=dict(np.load(D/'frozen_memory_seed.npz'));t=float(pose['time_s']);p.grid.set_source(src);p.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t);p.grid.seed(seed,t)
 obs=PolicyObservation(proprio_history=history['proprio_history'],action_history=history['action_history'],quaternion_world_wxyz_history=history['quaternion_world_wxyz_history'],stereo_left_rgb=images['left_rgb'],stereo_right_rgb=images['right_rgb'],position_world_xy=pose['root_position_world_m'][:2],yaw_world_rad=0.)
 gyro=json.loads((D/'ideal_rolling_sensors.json').read_text())[-1]['gyro_body_rad_s'];obs,goal=EstimatedNavigationObservation().build(obs,pose,gyro,np.array([2.2,.15,.45]));p.set_source_packet(dict(src,world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int)));p.set_current_packet(dict(frame=140,time_s=t,points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m']));return p,obs,goal
if __name__=='__main__':
 p,obs,goal=reconstruct()
 try:p.act(obs,goal)
 except Exception:
  error=traceback.format_exc();(H/'four_point_runtime_failure_CPU_trace_v2.txt').write_text(error);print(error)
 else:raise AssertionError('actual error not reproduced')
