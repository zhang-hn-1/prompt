import json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).parent
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
status=[]
for episode in ('hard_0_e1_0003','hard_0_e1_0004'):
    root=HERE/'baseline'/episode
    assert not root.exists(),'preserve actual results'
    cmd=[sys.executable,str(HERE/'run_baseline.py'),'--manifest',str(PROJECT/'configs/benchmark_v1/hard_0_dev_manifest.json'),
        '--split','dev','--episode-id',episode,'--policy','goal_greedy','--scene-root','/home/an/project/NavDP/assets/scenes',
        '--data-root',str(PROJECT/'data'),'--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--headless','--record-video']
    print('Starting',episode,flush=True)
    with (HERE/f'baseline_{episode}.log').open('w') as f:
        result=subprocess.run(cmd,env=dict(os.environ,PYTHONPATH=str(PROJECT),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_visual_hard0'),stdout=f,stderr=subprocess.STDOUT)
    status.append(dict(episode=episode,exit_code=result.returncode,command=cmd))
    (HERE/'baseline_run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('Finished',episode,result.returncode,flush=True)
    assert (root/f'episode_{episode}/metrics.json').exists(),'no actual outcome'
