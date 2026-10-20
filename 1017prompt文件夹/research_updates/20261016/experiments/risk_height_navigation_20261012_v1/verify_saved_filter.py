from pathlib import Path
import sys,json,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];sys.path[:0]=[str(D),str(W/'experiments/stereo_memory_navigation_20261004_v1'),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from height_interval import HeightIntervalSurfaceMemory
from stereo_geometry import restore_native
R=Path(sys.argv[1]) if len(sys.argv)>1 else W/'experiments/nearfield_rig_navigation_20261012_v1/candidate_e2_run'
m=HeightIntervalSurfaceMemory();n=0;total=0;removed=0
for p in sorted((R/'sensor_packets').glob('*.npz')):
 packet=dict(np.load(p));name=p.name.replace('source_','frame_');rgb=np.load(R/'source_inputs'/name);pred=np.load(R/'source_predictions'/name);base=np.load(R/'baseline_partial_observations'/name)
 result=m.update(packet,rgb['left_rgb'],rgb['right_rgb'],float(packet['time_s']),{},restore_native(pred['disparity_px']))
 keep=base['world'][:,2]-base['sigma_m']>.05
 for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:np.testing.assert_array_equal(result[k],base[k][keep])
 if R.parent==D:
  actual=np.load(R/'partial_observations'/name)
  for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:np.testing.assert_array_equal(result[k],actual[k])
 n+=1;total+=len(base['world']);removed+=int((~keep).sum())
assert n>0
result=dict(run=str(R),pairs_verified=n,input_accepted_points=total,interval_ambiguous_removed=removed,remaining_points=total-removed,actual_candidate_output_verified=R.parent==D,future_truth_read=False,baseline_aligned_replay_exact=True)
out=D/('actual_filter_verification.json' if R.parent==D else 'saved_filter_verification.json');out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
