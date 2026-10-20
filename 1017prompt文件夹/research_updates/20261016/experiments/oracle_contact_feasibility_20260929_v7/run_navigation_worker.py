"""Formal navigation using explicit local experimental inverse and frozen bank PID."""
import json,sys,hashlib
from pathlib import Path
import scripts.run_pointgoal_benchmark as runner
from roll_pid import install_runtime
from bank_mapping import BankMap,make_policy
from log_before_close import wrap_runtime
from margin_runtime import with_planning_margin
from course_geometry import with_course_geometry
from contact_audit import with_contact_audit
h=Path(__file__).resolve().parent;cfg=json.loads((h/'selected_candidate.json').read_text());model=BankMap(json.loads((h/'bank_mapping.json').read_text()));args=runner._build_parser().parse_args()
runner.IsaacBenchmarkRuntime=with_contact_audit(runner.IsaacBenchmarkRuntime,args.run_root/'feedback')
runner.IsaacBenchmarkRuntime=with_planning_margin(runner.IsaacBenchmarkRuntime,.10,args.run_root/'feedback')
runner.IsaacBenchmarkRuntime=install_runtime(runner.IsaacBenchmarkRuntime,**cfg['gains'],output=args.run_root/'feedback')
runner.OraclePathPolicy=make_policy(with_course_geometry(runner.OraclePathPolicy),model)
runner.OraclePathPolicy.policy_name='oracle_course_bank_pid_experimental'
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
(args.run_root/'experimental_controller.json').write_text(json.dumps(dict(planning_safety_margin_m=.10,collision_ball_radius_m=.40,configuration=cfg,base_policy='benchmark.policies.OraclePathPolicy',mapping='bank_mapping.make_policy override of tracking request only',execution='frozen roll_pid.FeedbackSmoother at50Hz',diagnostics='old V1 intermediate fields are pre-replacement; feedback/mapping_*.json contains effective mapping',files={str(h/n):hashlib.sha256((h/n).read_bytes()).hexdigest() for n in ['bank_mapping.py','bank_mapping.json','roll_pid.py','run_navigation_worker.py','log_before_close.py','selected_candidate.json','margin_runtime.py','course_geometry.py','contact_audit.py']}),indent=2))
raise SystemExit(runner.main())
