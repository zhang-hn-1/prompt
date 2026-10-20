"""Run one isolated v11 episode with the frozen v9 benchmark arguments."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / 'oracle_feasible_route_20260929_v9'
WM = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(case):
    original = json.loads((PREVIOUS / f'hybrid_hard_{case}_command.json').read_text())
    old_worker = str(PREVIOUS / 'hybrid/run_navigation_worker.py')
    old_root = str(PREVIOUS / f'navigation_hybrid_hard_{case}')
    worker = str(HERE / 'hybrid/run_navigation_worker.py')
    run_root = str(HERE / f'navigation_runtime_cost_{case}')
    assert old_worker in original and old_root in original
    cmd = [worker if x == old_worker else run_root if x == old_root else x for x in original]
    assert not Path(run_root).exists(), 'do not overwrite a run'
    (HERE / f'{case}_command.json').write_text(json.dumps(cmd, indent=2))
    files = sorted((HERE / 'hybrid').glob('*.py')) + [HERE / 'hybrid/bank_mapping.json', HERE / 'hybrid/selected_candidate.json']
    identity = {str(path): digest(path) for path in files}
    (HERE / f'{case}_source_identity.json').write_text(json.dumps(identity, indent=2))
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='2',
               MKL_NUM_THREADS='2', MPLCONFIGDIR='/tmp/ball_wm_mpl',
               PYTHONPATH='/home/an/.cache/ball_wm_visibility_numba_0612:' + str(WM) + ':' + str(WM / 'scripts'))
    start = time.time()
    with (HERE / f'{case}.log').open('w') as log:
        result = subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                cwd=WM.parent.parent)
    status = dict(case=case, returncode=result.returncode, wall_s=time.time() - start)
    (HERE / f'{case}_status.json').write_text(json.dumps(status, indent=2))
    (HERE / f'{case}_source_identity_after.json').write_text(json.dumps(
        {path: digest(Path(path)) == value for path, value in identity.items()}, indent=2))
    print(json.dumps(status), flush=True)


if __name__ == '__main__':
    main(sys.argv[1])
