import os,subprocess,sys,json
from pathlib import Path
here=Path(__file__).resolve().parent
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',PYTHONPATH='/home/an/.cache/ball_wm_visibility_numba_0612:/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm:/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/scripts')
result=[]
for case in ['0002','0004']:
    for condition in ['real','aligned']:
        name=f'{case}_{condition}'
        command=[sys.executable,str(here/'probe.py'),'--case',case,'--condition',condition,'--out',str(here/name)]
        print('START',name,flush=True)
        with (here/f'{name}.log').open('w') as log:
            p=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        result.append(dict(case=case,condition=condition,returncode=p.returncode,command=command))
        (here/'matrix_status.json').write_text(json.dumps(result,indent=2))
        print('DONE',name,p.returncode,flush=True)
        if p.returncode or not (here/name/'status.json').exists():sys.exit(1)
