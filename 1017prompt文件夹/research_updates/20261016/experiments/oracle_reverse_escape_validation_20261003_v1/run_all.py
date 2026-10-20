"""One run per frozen episode; sequential GPU use, no silent retries."""
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
PROJECT = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
env = dict(os.environ, PYTHONPATH=str(PROJECT), MPLCONFIGDIR='/tmp/mpl_reverse_validation_v1')
status = []
for i in (4,1,2,3):
    episode = f'hard_0_e1_{i:04}'
    root = HERE/episode
    if root.exists():
        raise RuntimeError(f'refusing to overwrite {root}')
    command = [sys.executable,str(HERE/'run_validation.py'),
               '--manifest',str(PROJECT/'configs/benchmark_v1/hard_0_dev_manifest.json'),
               '--split','dev','--episode-id',episode,'--policy','goal_greedy',
               '--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(PROJECT/'data'),
               '--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--headless','--record-video']
    print(f'Starting {episode}',flush=True)
    with (HERE/f'{episode}.log').open('w') as log:
        result = subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    status.append({'episode_id':episode,'returncode':result.returncode,'command':command})
    (HERE/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print(f'Finished {episode}: process exit {result.returncode}',flush=True)
    if not (root/f'episode_{episode}/metrics.json').exists():
        raise RuntimeError(f'{episode} produced no metrics; inspect log before continuing')
