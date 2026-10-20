"""Post-rejection CPU ablation only; NOT an accepted candidate or longer episode."""
import dataclasses,json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import reserve_planner
from source_map import SourceMapReplanner,ROOT,OLD
from benchmark.drive_model import DriveModel
from scripts.run_round4_fixed_reference import load_transformed_reference
HERE=Path(__file__).resolve().parent
reserve_planner.ESTIMATED_TIME_BUDGET_S=float('inf')
model=DriveModel.from_json(ROOT/'research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json')
for case in ['0002','0004']:
    ref,_=load_transformed_reference(OLD/f'episode_hard_0_e1_{case}/states.npz',case_id=case)
    adapter=SourceMapReplanner(case,model);start=time.perf_counter();result=adapter(SimpleNamespace(position_world_xy=ref.points_xy[0],yaw_world_rad=float(ref.yaw_rad[0])),ref.points_xy[-1])
    report=dict(case=case,diagnostic_only=True,condition='CPU same margin/time cost/node budget, no estimated-time pruning; no physics episode',result=dataclasses.asdict(result),seconds=time.perf_counter()-start,map=adapter.provenance)
    output=HERE/f'{case}_diagnostic_no_time_cutoff.json'
    if output.exists():raise RuntimeError('refuse overwrite')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(case,result.status,result.expanded_nodes,result.estimated_time_s,flush=True)
