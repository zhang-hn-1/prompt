import subprocess,os,sys,json,hashlib,time
from pathlib import Path
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
(h/'dynamic_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in h.glob('*') if p.suffix in ('.py','.json','.md')},indent=2))
status=[]
for limit,tag in [('0.60','limit060'),('none','limitoff')]:
 for main in [.5,-1.5]:
  name=f'{tag}_{main}';cmd=[sys.executable,str(h/'run_probe.py'),'--main',str(main),'--mode','pid','--tag',tag,'--gains','.4,.1,.7','--fresh','--limit',limit];print('START',name,flush=True);t=time.time()
  with (h/(name+'.log')).open('w') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
  status.append(dict(case=name,limit=limit,returncode=r.returncode,wall_s=time.time()-t));(h/'dynamic_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
  if r.returncode:sys.exit(r.returncode)
