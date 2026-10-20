import dataclasses,json,time,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from source_map import SourceMapReplanner,ROOT,OLD
from benchmark.drive_model import DriveModel
from scripts.run_round4_fixed_reference import load_transformed_reference
HERE=Path(__file__).resolve().parent
model=DriveModel.from_json(ROOT/'research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json')
for case in ['0002','0004']:
    ref,_=load_transformed_reference(OLD/f'episode_hard_0_e1_{case}/states.npz',case_id=case)
    fn=SourceMapReplanner(case,model);start=time.perf_counter()
    result=fn(SimpleNamespace(position_world_xy=ref.points_xy[0],yaw_world_rad=float(ref.yaw_rad[0])),ref.points_xy[-1])
    report=dict(case=case,result=dataclasses.asdict(result),seconds=time.perf_counter()-start,map=fn.provenance,original_goal=ref.points_xy[-1].tolist(),candidate_sha256=hashlib.sha256((HERE/'reserve_planner.py').read_bytes()).hexdigest())
    (HERE/f'{case}_plan.json').write_text(json.dumps(report,indent=2)+'\n')
    if result.status=='success':
        np.savez(HERE/f'{case}_reference.npz',kinodynamic_reference_pose=[(w.x_m,w.y_m,w.yaw_rad) for w in result.waypoints],kinodynamic_reference_command=[(w.main_command_rad_s,w.auxiliary_angle_rad) for w in result.waypoints],kinodynamic_reference_drive_direction=[1 if w.drive_direction=='forward' else -1 for w in result.waypoints])
    print(case,result.status,result.expanded_nodes,result.path_length_m,report['seconds'],flush=True)
