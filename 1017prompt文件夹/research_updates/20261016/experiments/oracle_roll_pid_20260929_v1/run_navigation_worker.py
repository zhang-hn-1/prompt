"""Formal runner plus explicitly identified experimental bank-feedback execution."""
import json,sys,hashlib
from pathlib import Path
import scripts.run_pointgoal_benchmark as runner
from roll_pid import install_runtime
h=Path(__file__).resolve().parent
cfg=json.loads((h/'selected_candidate.json').read_text())
args=runner._build_parser().parse_args()
runner.IsaacBenchmarkRuntime=install_runtime(runner.IsaacBenchmarkRuntime,**cfg['gains'],output=args.run_root/'feedback')
original_factory=runner._make_policy

def factory(*a,**kw):
 policy=original_factory(*a,**kw)
 policy.policy_name='oracle_bank_pid_experimental'
 return policy
runner._make_policy=factory
original_child=runner._child_command

def child(*a,**kw):
 command=original_child(*a,**kw);command[1]=str(Path(__file__).resolve());return command
runner._child_command=child
args.run_root.mkdir(parents=True,exist_ok=True)
(args.run_root/'experimental_controller.json').write_text(json.dumps(dict(configuration=cfg,base_policy='benchmark.policies.OraclePathPolicy',execution='roll_pid.FeedbackSmoother at 50Hz',protocol_command_semantics='original auxiliary request is feedforward and 0.75-scaled bank reference; PID auxiliary position is logged separately',files={str(h/n):hashlib.sha256((h/n).read_bytes()).hexdigest() for n in ['roll_pid.py','run_navigation_worker.py','selected_candidate.json']}),indent=2))
raise SystemExit(runner.main())
