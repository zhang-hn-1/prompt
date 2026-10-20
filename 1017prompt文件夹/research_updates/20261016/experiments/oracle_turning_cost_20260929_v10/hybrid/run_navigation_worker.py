"""Formal navigation using explicit local experimental inverse and frozen bank PID."""
import json,sys,hashlib
from pathlib import Path
import scripts.run_pointgoal_benchmark as runner
from roll_pid import install_runtime
from bank_mapping import BankMap,make_policy
from gated_speed import with_confident_speed
from log_before_close import wrap_runtime
import benchmark.isaac_runtime as runtime_module
from hybrid_planning import choose_reference
from time_planning import plan_kinodynamic_path as plan_time_feasible
from benchmark.kinodynamic_planning import KinodynamicPlanRequest
from benchmark.drive_model import DriveModel
import numpy as np
def candidate_reference(*, occupancy_grid, resolution_m, origin_xy, episode, drive_profile_path):
    q=episode.start_quaternion_world_wxyz
    w,x,y,z=(float(v) for v in q)
    yaw=__import__('math').atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    request=KinodynamicPlanRequest(
        occupancy_grid=occupancy_grid, resolution_m=resolution_m,
        origin_xy=tuple(np.asarray(origin_xy,dtype=np.float64)),
        start_xy=episode.start_position_world_m[:2], start_yaw_rad=yaw,
        goal_xy=episode.goal_position_world_m[:2],
    )
    return plan_time_feasible(request, DriveModel.from_json(drive_profile_path))

frozen_reference=runtime_module._plan_kinodynamic_reference

def selected_reference(**kwargs):
    old=frozen_reference(**kwargs)
    fresh=candidate_reference(**kwargs)
    selected,decision=choose_reference(old,fresh,DriveModel.from_json(kwargs['drive_profile_path']))
    out=args.run_root/'feedback'/'reference_decision.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(decision,indent=2))
    return selected

runtime_module._plan_kinodynamic_reference=selected_reference
h=Path(__file__).resolve().parent;cfg=json.loads((h/'selected_candidate.json').read_text());model=BankMap(json.loads((h/'bank_mapping.json').read_text()));args=runner._build_parser().parse_args()
runner.IsaacBenchmarkRuntime=install_runtime(runner.IsaacBenchmarkRuntime,**cfg['gains'],output=args.run_root/'feedback')
runner.OraclePathPolicy=with_confident_speed(make_policy(runner.OraclePathPolicy,model))
runner.OraclePathPolicy.policy_name='oracle_time_feasible_confident_speed_experimental'
original_factory=runner._make_policy;instances=[]
def factory(*a,**kw):
 p=original_factory(*a,**kw);instances.append(p);return p
runner._make_policy=factory
runner.IsaacBenchmarkRuntime=wrap_runtime(runner.IsaacBenchmarkRuntime,lambda: instances[-1] if instances else None,args.run_root/'feedback')
original_run=runner._run_one
def run_one(args,episode):
 try:return original_run(args,episode)
 finally:
  out=args.run_root/'feedback';out.mkdir(parents=True,exist_ok=True)
  if instances and hasattr(instances[-1],'mapping_records'):
   (out/('mapping_'+episode.episode_id+'.json')).write_text(json.dumps(instances[-1].mapping_records))
runner._run_one=run_one
original_child=runner._child_command
def child(*a,**kw):
 c=original_child(*a,**kw);c[1]=str(Path(__file__).resolve());return c
runner._child_command=child
args.run_root.mkdir(parents=True,exist_ok=True)
(args.run_root/'experimental_controller.json').write_text(json.dumps(dict(configuration=cfg,base_policy='benchmark.policies.OraclePathPolicy',mapping='bank_mapping.make_policy override of tracking request only',execution='frozen roll_pid.FeedbackSmoother at50Hz',diagnostics='old V1 intermediate fields are pre-replacement; feedback/mapping_*.json contains effective mapping',files={str(h/n):hashlib.sha256((h/n).read_bytes()).hexdigest() for n in ['bank_mapping.py','bank_mapping.json','roll_pid.py','run_navigation_worker.py','log_before_close.py','selected_candidate.json','time_planning.py','gated_speed.py','hybrid_planning.py']}),indent=2))
raise SystemExit(runner.main())
