import datetime,json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
p=json.loads((H/'large_bank_protocol.json').read_text());status=[]
for name in p['names']:
    command=[sys.executable,str(H/'run_large_bank.py'),'--name',name]
    row=dict(name=name,command=command,start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='running')
    status.append(row);(H/'large_bank_status.json').write_text(json.dumps(status,indent=2)+'\n')
    with (H/f'large_bank_{name}.log').open('w') as stream:
        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(P),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
    row.update(exit_code=result.returncode,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='finished')
    (H/'large_bank_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISHED',name,result.returncode,flush=True)
    if result.returncode:raise RuntimeError(name)
