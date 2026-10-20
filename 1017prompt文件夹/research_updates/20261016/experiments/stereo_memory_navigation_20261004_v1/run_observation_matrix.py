import os,subprocess,sys,json,datetime
from pathlib import Path
H=Path(__file__).resolve().parent
status=[]
for name in ('zero','bank','forward','reverse'):
    print('START',name,flush=True)
    start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    with (H/f'observation_{name}.log').open('w') as f:
        r=subprocess.run([sys.executable,str(H/'run_observation.py'),'--name',name],stdout=f,stderr=subprocess.STDOUT,
                         env=dict(os.environ,OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
    row=dict(name=name,exit_code=r.returncode,start_utc=start,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    status.append(row);(H/'observation_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISH',name,r.returncode,flush=True)
    if r.returncode:raise RuntimeError(name)
print('FOUR_ACTUAL_OBSERVATIONS_COMPLETE',flush=True)
