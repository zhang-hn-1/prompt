from common import *
import os, sys
sys.path.insert(0, str(W / 'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg = load(D / 'protocol.json')
for p, h in cfg['code_sha256'].items(): assert sha(p) == h, p
assert not (D / 'ledger.json').exists()
ledger = dict(dispatch_cap=1, dispatch_used=1, recovery_cap=1, recovery_used=0,
    exposures_cap=470, evaluation_times_cap=1910, fit=0, learned_inference=0,
    source_calls=0, physics=0, navigation=0, old_budgets_reopened=False, parent_pid=os.getpid())
put(D / 'ledger.json', ledger)
r = run_group([sys.executable, '-B', str(D / 'stage.py')], D / 'status.json', D / 'run.log',
    timeout=300, grace=15, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1'))
ledger.update(status=r['status'], execution=r); put(D / 'ledger.json', ledger)
assert r['status'] == 'finished' and r['child_returncode'] == 0, r
