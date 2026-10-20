from pathlib import Path
import sys, os, json, hashlib, datetime
D = Path(__file__).resolve().parent; W = D.parents[1]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
cfg = json.loads((D / 'protocol.json').read_text())
for section in ['input_sha256', 'code_sha256']:
    for p, h in cfg[section].items(): assert sha(p) == h, p
ledger_path = D / 'ledger.json'; assert not ledger_path.exists()
ledger = dict(dispatch_cap=1, dispatch_used=1, recovery_cap=0, recovery_used=0,
              source_compute_replays_cap=8, matcher_calls_cap=32, fit=0, physics=0, navigation=0,
              parent_pid=os.getpid(), utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), status='reserved')
ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=cfg['sdk'],
           LD_LIBRARY_PATH=cfg['sdk'] + '/bin' + (':' + os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else ''),
           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
sys.path.insert(0, str(W / 'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
status = run_group([sys.executable, '-B', str(D / 'diagnose.py')], D / 'status.json', D / 'run.log', timeout=90, grace=10, env=env)
ledger.update(status=status['status'], execution=status)
ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
assert status['status'] == 'finished' and status['child_returncode'] == 0, status
