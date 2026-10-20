import os,subprocess,sys,json
from pathlib import Path
here=Path(__file__).resolve().parent
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',PYTHONPATH='/home/an/.cache/ball_wm_visibility_numba_0612:/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm:/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/scripts')
results=[]
for case,condition in [('0002','real'),('0002','candidate'),('0004','real'),('0004','candidate')]:
    name=f'{case}_real' if condition=='real' else f'{case}_candidate_source_map'
    if (here/name/'status.json').exists():
        print('KEEP COMPLETED',name,flush=True);continue
    command=[sys.executable,str(here/'probe.py'),'--case',case,'--condition',condition,'--out',str(here/name)]
    print('START',case,flush=True)
    with (here/f'{name}.log').open('w') as log:p=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    results.append(dict(command=command,returncode=p.returncode))
    (here/'candidate_source_map_status.json').write_text(json.dumps(results,indent=2))
    print('DONE',case,p.returncode,flush=True)
    if p.returncode:sys.exit(1)
