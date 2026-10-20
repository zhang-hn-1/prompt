import json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).parent
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
status=[]
for label in ('e0','e1','e2'):
    root=HERE/'legacy_clean'/label
    assert not root.exists(),'never overwrite results'
    cmd=[sys.executable,str(HERE/'run_clean_legacy.py'),'--manifest',str(PROJECT/'configs/benchmark_v1/golden_v1_manifest.json'),
        '--split','golden','--episode-id',f'golden_{label}_0001','--policy','goal_greedy',
        '--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(PROJECT/'data'),'--run-root',str(root),
        '--lighting-profile','fixed_camera_fill_v1','--headless','--record-video']
    print('Starting',label,flush=True)
    with (HERE/f'legacy_{label}.log').open('w') as f:
        result=subprocess.run(cmd,env=dict(os.environ,PYTHONPATH=str(PROJECT),OPENBLAS_NUM_THREADS='1'),stdout=f,stderr=subprocess.STDOUT)
    status.append(dict(label=label,exit_code=result.returncode,command=cmd))
    (HERE/'legacy_run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('Finished',label,result.returncode,flush=True)
    assert (root/f'episode_golden_{label}_0001/metrics.json').exists(),'actual simulator outcome missing'
