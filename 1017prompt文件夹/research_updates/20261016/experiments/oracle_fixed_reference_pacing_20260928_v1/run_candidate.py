import os,subprocess,sys,json,hashlib
from pathlib import Path
here=Path(__file__).resolve().parent
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',PYTHONPATH='/home/an/.cache/ball_wm_visibility_numba_0612:/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm:/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/scripts')
paths=list(here.glob('*.py'))+list(here.glob('*_reference.npz'))+[here/'PROTOCOL.md']
root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
paths += [root/x for x in ['benchmark/policies.py','benchmark/isaac_runtime.py','scripts/action_smoothing.py','scripts/run_tracking_acceptance.py','scripts/run_round4_fixed_reference.py','configs/data_contract.yaml','configs/benchmark_v1/drive_dynamics_v1.json']]
identity={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
(here/'pre_run_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
results=[]
for case in ['0002','0004']:
    if json.loads((here/f'{case}_plan.json').read_text())['result']['status']!='success':
        raise RuntimeError('Both geometric plans must pass before simulation')
for case,condition in [('0002','real'),('0002','candidate'),('0004','real'),('0004','candidate')]:
    name=f'{case}_real' if condition=='real' else f'{case}_reserve'
    if (here/name).exists():raise RuntimeError('Refusing existing output directory')
    command=[sys.executable,str(here/'probe.py'),'--case',case,'--condition',condition,'--out',str(here/name)]
    print('START',case,flush=True)
    with (here/f'{name}.log').open('w') as log:p=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    results.append(dict(command=command,returncode=p.returncode))
    (here/'reserve_status.json').write_text(json.dumps(results,indent=2))
    print('DONE',case,p.returncode,flush=True)
    if p.returncode:sys.exit(1)
