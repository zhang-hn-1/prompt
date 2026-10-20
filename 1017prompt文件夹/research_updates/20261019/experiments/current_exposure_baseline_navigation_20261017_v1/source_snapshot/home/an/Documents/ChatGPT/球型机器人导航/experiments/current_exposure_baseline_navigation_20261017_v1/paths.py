from pathlib import Path
import sys
D=Path(__file__).resolve().parent; W=D.parents[1]
H=W/'experiments/stereo_memory_navigation_20261004_v1'
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
SDK=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
PATHS=[D,P,H,W/'experiments/user_fov_position_only_20261015_v1',W/'experiments/oracle_feasible_route_20260929_v9/hybrid',W/'experiments/bilateral_temporal_geometry_navigation_20261003_v1',W/'experiments/clean_visual_hard0_intervention_20261003_v1',W/'experiments/visible_surface_connectivity_20261004_v1']
sys.path[:0]=list(map(str,PATHS))
