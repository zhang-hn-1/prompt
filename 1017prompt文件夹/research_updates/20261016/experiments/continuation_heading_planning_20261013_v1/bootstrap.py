from pathlib import Path
import sys
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path[:0]=list(map(str,[D,V,P,H,W/'experiments/oracle_feasible_route_20260929_v9/hybrid',W/'experiments/bilateral_temporal_geometry_navigation_20261003_v1',W/'experiments/clean_visual_hard0_intervention_20261003_v1',W/'experiments/visible_surface_connectivity_20261004_v1']))
