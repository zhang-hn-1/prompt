"""Plan bounded hard_0 routes with the controller-tier time edge cost."""

import json
import math
from pathlib import Path
import time

import numpy as np

from benchmark.drive_model import DriveModel
from benchmark.grid_planning import inflate_obstacles
from benchmark.kinodynamic_planning import KinodynamicPlanRequest
from benchmark.manifest import load_manifest
from benchmark.task_cards import load_occupancy_npz
from scripts.run_pointgoal_benchmark import _resolve_occupancy_path
from time_planning import plan_kinodynamic_path, scheduled_primitive_time_s


HERE = Path(__file__).resolve().parent
WM = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
manifest = load_manifest(WM / 'configs/benchmark_v1/hard_0_dev_manifest.json')
model = DriveModel.from_json(WM / 'configs/benchmark_v1/drive_dynamics_v1.json')
results = {}
for suffix in ('0001', '0002', '0003', '0004'):
    case = f'hard_0_e1_{suffix}'
    episode = next(e for e in manifest.episodes if e.episode_id == case)
    path = _resolve_occupancy_path(WM / 'configs/benchmark_v1/scene_index.json', episode, WM / 'data')
    occupancy = load_occupancy_npz(path)
    grid = inflate_obstacles(np.where(occupancy.grid == 0, 0, 1).astype(np.uint8),
                             radius_m=.5, resolution_m=occupancy.resolution_m, occupied=1)
    w, x, y, z = episode.start_quaternion_world_wxyz
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    request = KinodynamicPlanRequest(
        occupancy_grid=grid, resolution_m=occupancy.resolution_m,
        origin_xy=tuple(occupancy.global_origin_xy),
        start_xy=episode.start_position_world_m[:2],
        start_yaw_rad=yaw, goal_xy=episode.goal_position_world_m[:2],
    )
    start = time.time()
    result = plan_kinodynamic_path(request, model)
    entry = dict(status=result.status, expanded_nodes=result.expanded_nodes,
                 search_wall_s=time.time()-start,
                 length_m=result.path_length_m,
                 direction_changes=result.direction_changes)
    if result.status == 'success':
        actions = [(p.main_command_rad_s, p.auxiliary_angle_rad) for p in result.waypoints[:-1]]
        entry['scheduled_steady_time_s'] = float(sum(
            scheduled_primitive_time_s(main, aux, .3, model)
            for main, aux in actions
        ) + result.direction_changes)
        entry['naive_steady_time_s'] = float(sum(
            .3/model.speed(main, aux) for main, aux in actions
        ) + result.direction_changes)
        entry['high_reference_curvature_segments'] = sum(
            abs(model.curvature(main, aux)) >= .55 for main, aux in actions
        )
        entry['waypoints'] = [dict(x_m=p.x_m, y_m=p.y_m, yaw_rad=p.yaw_rad,
                                  direction=p.drive_direction,
                                  main=p.main_command_rad_s,
                                  auxiliary=p.auxiliary_angle_rad)
                              for p in result.waypoints]
    results[case] = entry
    (HERE / 'offline_results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(case, entry['status'], entry['length_m'], entry.get('scheduled_steady_time_s'),
          entry['direction_changes'], entry.get('high_reference_curvature_segments'),
          'search', round(entry['search_wall_s'], 1), flush=True)
