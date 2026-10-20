"""Prepared saved-only score recovery; direct human exception required before use."""
from pathlib import Path
import hashlib, json, os, time

D = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    proposal_path = D / 'proposed_score_only_recovery.json'
    approval_path = D / 'score_recovery_approval.json'
    if not approval_path.is_file():
        raise RuntimeError('No direct human score-only recovery exception')
    approval = json.loads(approval_path.read_text())
    assert approval.get('user_explicit_authorization') is True
    assert approval['proposal_sha256'] == sha(proposal_path)
    cfg = json.loads(proposal_path.read_text())
    for p, h in cfg['execution_identity'].items():
        assert sha(p) == h, p
    ledger = json.loads((D / 'score_recovery_ledger.json').read_text())
    assert ledger['cap'] == ledger['used'] == 1 and ledger['recovery_after_this'] == 0
    assert ledger['status'] == 'reserved' and ledger['parent_pid'] == os.getppid()
    assert ledger['proposal_sha256'] == sha(proposal_path)
    registered = False
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        try:
            status = json.loads((D / 'score_recovery_status.json').read_text())
            registered = status['status'] == 'running' and status['child_pid'] == os.getpid() and status['parent_pid'] == os.getppid()
        except (FileNotFoundError, json.JSONDecodeError):
            registered = False
        if registered:
            break
        time.sleep(.01)
    assert registered, 'Durable recovery child PID registration required'
    manifest = json.loads((D / 'raw_artifact_preservation_manifest.json').read_text())
    for p, row in manifest['files'].items():
        assert sha(p) == row['sha256'], p
    original = D / 'score_C.py'
    assert sha(original) == cfg['original_score_sha256']
    source = original.read_text()
    needle = "assert a['requests'].shape==(actions,4)"
    assert source.count(needle) == 1
    patched = source.replace(needle, "assert a['requests'].shape==(actions,10)")
    assert hashlib.sha256(patched.encode()).hexdigest() == cfg['proposed_patched_score_sha256']
    # Original file, frozen hashes, failed ledger and raw outputs remain unchanged.
    exec(compile(patched, str(original) + ' [approved structural recovery]', 'exec'),
         {'__name__': '__main__', '__file__': str(original)})

if __name__ == '__main__':
    main()
