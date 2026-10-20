import subprocess,os,sys,json,time,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts');status=[]
for speed in [1.0,-1.0]:
 name=f'validation_{speed}';print('START',name,flush=True);t=time.time()
 with (h/(name+'.log')).open('w') as f:r=subprocess.run([sys.executable,str(h/'run_validation.py'),'--main',str(speed),'--mode','pid'],env=env,stdout=f,stderr=subprocess.STDOUT)
 status.append(dict(case=name,returncode=r.returncode,wall_s=time.time()-t));(h/'validation_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
 if r.returncode:sys.exit(r.returncode)
