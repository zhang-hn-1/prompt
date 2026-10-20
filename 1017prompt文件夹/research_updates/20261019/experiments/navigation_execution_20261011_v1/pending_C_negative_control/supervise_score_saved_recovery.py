"""Prepared one-dispatch saved-only recovery supervisor. Never launches physics."""
from pathlib import Path
import datetime, hashlib, json, os, sys

D = Path(__file__).resolve().parent
W = D.parents[2]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()

def put(p, value):
    p.write_text(json.dumps(value, indent=2) + '\n')

def main():
    proposal_path = D / 'proposed_score_only_recovery.json'
    approval_path = D / 'score_recovery_approval.json'
    if not approval_path.is_file():
        raise RuntimeError('Direct human score-only recovery exception not received')
    approval = json.loads(approval_path.read_text())
    assert approval.get('user_explicit_authorization') is True
    assert approval['proposal_sha256'] == sha(proposal_path)
    cfg = json.loads(proposal_path.read_text())
    assert cfg['score_dispatch_cap'] == cfg['recovery_cap'] == 1
    assert cfg['physics'] == cfg['navigation'] == cfg['source_calls'] == cfg['fit'] == 0
    for p, h in cfg['execution_identity'].items():
        assert sha(p) == h, p
    nav = json.loads((D / 'navigation_status.json').read_text())
    failed = json.loads((D / 'score_status.json').read_text())
    assert nav['status'] == 'finished' and nav['child_returncode'] == 0
    assert failed['status'] == 'runtime_failed' and failed['child_returncode'] == 1
    assert not nav['remaining_live_group_pids'] and not failed['remaining_live_group_pids']
    ledger_path = D / 'score_recovery_ledger.json'
    assert not ledger_path.exists() and not (D / 'score_recovery_status.json').exists()
    assert not (D / 'e1_result.json').exists() and not (D / 'e1_run/artifact_manifest_post_close.json').exists()
    ledger = dict(cap=1, used=1, recovery_after_this=0, status='reserved', parent_pid=os.getpid(),
                  utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  proposal_sha256=sha(proposal_path), approval_sha256=sha(approval_path))
    with ledger_path.open('x') as stream:
        json.dump(ledger, stream, indent=2)
    old = json.loads((D / 'execution_protocol.json').read_text())
    sdk = old['sdk']
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=sdk,
               LD_LIBRARY_PATH=sdk + '/bin' + (':' + os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else ''),
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', DISPLAY=':0', XDG_RUNTIME_DIR='/run/user/1000',
               MPLCONFIGDIR='/tmp/mpl_C_negativecontrol', TORCH_COMPILE_DISABLE='1')
    sys.path.insert(0, str(W / 'experiments/fixed_source_parallel_compute_20261017_v1'))
    from group_supervision import run_group
    status = run_group([sys.executable, '-B', str(D / 'score_saved_recovery.py')],
                       D / 'score_recovery_status.json', D / 'score_recovery.log',
                       timeout=cfg['timeout_s'], grace=15, env=env)
    ledger.update(status=status['status'], execution=status)
    put(ledger_path, ledger)
    assert status['status'] == 'finished' and status['child_returncode'] == 0, status

if __name__ == '__main__':
    main()
