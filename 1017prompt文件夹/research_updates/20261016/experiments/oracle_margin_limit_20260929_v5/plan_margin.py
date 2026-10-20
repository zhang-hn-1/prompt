from pathlib import Path
import json,time,dataclasses,numpy as np
from benchmark.manifest import load_manifest
from benchmark.task_cards import load_occupancy_npz
from benchmark.grid_planning import inflate_obstacles
from benchmark.isaac_runtime import _plan_kinodynamic_reference
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');occ=load_occupancy_npz(wm/'data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz');episodes=load_manifest(wm/'configs/benchmark_v1/hard_0_dev_manifest.json').episodes;out=[]
for e in episodes:
 if e.episode_id not in ['hard_0_e1_0002','hard_0_e1_0004']:continue
 for margin in [.1,.2]:
  t=time.time();grid=inflate_obstacles(np.where(occ.grid==0,0,1).astype(np.uint8),radius_m=.4+margin,resolution_m=occ.resolution_m,occupied=1);p=_plan_kinodynamic_reference(occupancy_grid=grid,resolution_m=occ.resolution_m,origin_xy=occ.global_origin_xy,episode=e,drive_profile_path=wm/'configs/benchmark_v1/drive_dynamics_v1.json');row=dict(episode=e.episode_id,margin=margin,wall_s=time.time()-t,result=dataclasses.asdict(p));out.append(row);print(e.episode_id,margin,p.status,p.path_length_m,flush=True)
  if p.status=='success':
   pose=np.array([(w.x_m,w.y_m,w.yaw_rad) for w in p.waypoints]);np.savez(h/f'plan_{e.episode_id}_{margin}.npz',pose=pose)
   if margin==.1:assert np.array_equal(pose,np.load(h.parent/f'oracle_pid_validation_20260929_v4/navigation_hard_0/episode_{e.episode_id}/states.npz')['kinodynamic_reference_pose'])
  (h/'margin_plans.json').write_text(json.dumps(out,indent=2))
