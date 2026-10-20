"""Run the single lower-curvature candidate under the frozen 35 s protocol."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / 'oracle_feasible_route_20260929_v9'
WORKER = HERE / 'hybrid/run_navigation_worker.py'
CASES = ('hard_0_e1_0002', 'hard_0_e1_0004')
RUN_TAG = 'gpu'


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    source_files = sorted((HERE / 'hybrid').glob('*.py'))
    source_files += [HERE / 'hybrid/bank_mapping.json', HERE / 'hybrid/selected_candidate.json']
    identity = {str(p): sha(p) for p in source_files}
    (HERE / 'source_identity.json').write_text(json.dumps(identity, indent=2))
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='2',
               MKL_NUM_THREADS='2', MPLCONFIGDIR='/tmp/ball_wm_mpl',
               PYTHONPATH='/home/an/.cache/ball_wm_visibility_numba_0612:'
               '/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm:'
               '/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/scripts')
    status = []
    for case in CASES:
        original = json.loads((PREVIOUS / f'hybrid_hard_{case}_command.json').read_text())
        cmd = [str(WORKER) if arg == str(PREVIOUS / 'hybrid/run_navigation_worker.py')
               else str(HERE / f'navigation_cap055_{case}_{RUN_TAG}') if arg == str(PREVIOUS / f'navigation_hybrid_hard_{case}')
               else arg for arg in original]
        assert str(WORKER) in cmd
        assert str(HERE / f'navigation_cap055_{case}_{RUN_TAG}') in cmd
        (HERE / f'{case}_{RUN_TAG}_command.json').write_text(json.dumps(cmd, indent=2))
        print('START', case, flush=True)
        start = time.time()
        with (HERE / f'{case}_{RUN_TAG}.log').open('w') as log:
            result = subprocess.run(
                cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                cwd='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark',
            )
        status.append({'case': case, 'returncode': result.returncode,
                       'wall_s': time.time() - start})
        (HERE / f'status_{RUN_TAG}.json').write_text(json.dumps(status, indent=2))
        print('DONE', case, result.returncode, flush=True)
        if result.returncode:
            break
    (HERE / 'source_identity_after.json').write_text(json.dumps(
        {path: sha(Path(path)) == checksum for path, checksum in identity.items()}, indent=2))


if __name__ == '__main__':
    main()
