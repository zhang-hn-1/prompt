"""Exactly four full independent resets; one unchanged visual candidate."""
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
status=[]
for difficulty in ('e2','e1'):
    for mode in ('b0','candidate'):
        case=f'{mode}_{difficulty}';root=HERE/case
        if root.exists():raise RuntimeError(f'refuse overwrite: {root}')
        env=dict(os.environ,PYTHONPATH=str(PROJECT),OPENBLAS_NUM_THREADS='1',
                 MPLCONFIGDIR='/tmp/mpl_bilateral_temporal_v1',BILATERAL_TEMPORAL_MODE=mode)
        command=[sys.executable,str(HERE/'run_worker.py'),'--manifest',str(PROJECT/'configs/benchmark_v1/golden_v1_manifest.json'),
                 '--split','golden','--episode-id',f'golden_{difficulty}_0001','--policy','goal_greedy',
                 '--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(PROJECT/'data'),
                 '--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--headless','--record-video']
        print('Starting',case,flush=True)
        with (HERE/f'{case}.log').open('w') as log:
            result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        metric=root/f'episode_golden_{difficulty}_0001/metrics.json'
        status.append(dict(case=case,exit_code=result.returncode,command=command))
        (HERE/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
        print('Finished',case,'exit',result.returncode,flush=True)
        if not metric.exists():raise RuntimeError(f'no completed outcome for {case}; inspect actual log')
