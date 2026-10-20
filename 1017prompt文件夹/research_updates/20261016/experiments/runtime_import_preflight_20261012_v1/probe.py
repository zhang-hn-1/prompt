"""Bounded class/module import only. Never constructs runtime or SimulationApp."""
from pathlib import Path
import sys,json,importlib,importlib.util
W=Path(__file__).resolve().parents[2]
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
dirs=[P,P/'scripts',W/'experiments/stereo_memory_navigation_20261004_v1',W/'experiments/tube_bridge_20261012_v1',W/'experiments/tube_anchor_provider_20261012_v1',W/'experiments/tube_collector_20261012_v1',W/'experiments/tube_journal_20261012_v1',W/'experiments/execution_tube_qualification_20261012_v1',W/'experiments/tube_provenance_20261012_v1',W/'experiments/oracle_feasible_route_20260929_v9/hybrid',W/'experiments/bilateral_temporal_geometry_navigation_20261003_v1',W/'experiments/clean_visual_hard0_intervention_20261003_v1']
for d in dirs:sys.path.insert(0,str(d))
checks=[]
for name,attribute in [('scripts.run_pointgoal_benchmark','IsaacBenchmarkRuntime'),('native_rolling_shadow','NativeRollingShadow'),('flat_ground_pose_bridge','FlatGroundPoseBridge'),('imu_interval_packet','interval_packet'),('render_boundary','hide_diagnostic_markers'),('complete_capture','complete_capture'),('render_flush','fixed_render_flush'),('clock_sync','ensure_current_frames'),('roll_pid','install_runtime'),('block_bridge','BlockBridge'),('provider','provide_clocked')]:
 try:
  m=importlib.import_module(name);getattr(m,attribute);checks.append({'module':name,'source':str(m.__file__),'status':'loaded'})
 except Exception as e:checks.append({'module':name,'status':'import_failed','exception':type(e).__name__,'message':str(e)})
print(json.dumps({'checks':checks,'all_loaded':all(x['status']=='loaded' for x in checks),'runtime_constructed':False,'SimulationApp_started':False,'physics_runs':0,'fit_runs':0,'scope':'Current-host Python imports only, not SDK renderer/sensor creation or cross-machine execution'}))
if not all(x['status']=='loaded' for x in checks):raise SystemExit(2)
