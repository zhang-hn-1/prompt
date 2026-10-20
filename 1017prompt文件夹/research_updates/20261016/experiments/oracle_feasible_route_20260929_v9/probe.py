import json,math,time
from pathlib import Path
import numpy as np
from benchmark.manifest import load_manifest
from benchmark.drive_model import DriveModel
from benchmark.task_cards import load_occupancy_npz
from benchmark.grid_planning import inflate_obstacles
from benchmark.kinodynamic_planning import KinodynamicPlanRequest,plan_kinodynamic_path
from scripts.run_pointgoal_benchmark import _resolve_occupancy_path
from time_planning import plan_kinodynamic_path as candidate
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
manifest=load_manifest(wm/'configs/benchmark_v1/hard_0_dev_manifest.json');episode=next(x for x in manifest.episodes if x.episode_id=='hard_0_e1_0002')
op=_resolve_occupancy_path(wm/'configs/benchmark_v1/scene_index.json',episode,wm/'data')
occ=load_occupancy_npz(op);grid=np.where(occ.grid==0,0,1).astype(np.uint8);inflated=inflate_obstacles(grid,radius_m=.5,resolution_m=occ.resolution_m,occupied=1)
q=episode.start_quaternion_world_wxyz;w,x,y,z=q;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
r=KinodynamicPlanRequest(occupancy_grid=inflated,resolution_m=occ.resolution_m,origin_xy=tuple(occ.global_origin_xy),start_xy=episode.start_position_world_m[:2],start_yaw_rad=yaw,goal_xy=episode.goal_position_world_m[:2]);model=DriveModel.from_json(wm/'configs/benchmark_v1/drive_dynamics_v1.json')
results={}
for name,fn in [('frozen',plan_kinodynamic_path),('time_feasible',candidate)]:
 st=time.time();p=fn(r,model); print(name,p.status,p.expanded_nodes,p.path_length_m,p.direction_changes,time.time()-st,flush=True)
 if p.status!='success':results[name]=dict(status=p.status,expanded_nodes=p.expanded_nodes);continue
 coords=np.array([[v.x_m,v.y_m,v.yaw_rad] for v in p.waypoints]);actions=np.array([[v.main_command_rad_s,v.auxiliary_angle_rad] for v in p.waypoints[:-1]]);ks=np.array([model.curvature(*a) for a in actions]);speeds=np.array([model.speed(*a) for a in actions]);times=.3/speeds;estimated=float(times.sum()+1.0*p.direction_changes)
 results[name]=dict(status=p.status,expanded_nodes=p.expanded_nodes,path_length_m=p.path_length_m,direction_changes=p.direction_changes,estimated_steady_time_s=estimated,reference_max_abs_curvature_1_m=float(max(abs(ks))),reference_steps_main_0p5=int(sum(abs(actions[:,0])==.5)),reference_steps_main_1p5=int(sum(abs(actions[:,0])==1.5)),waypoints=coords.tolist(),actions=actions.tolist())
(h/'offline_results.json').write_text(json.dumps(results,indent=2))
