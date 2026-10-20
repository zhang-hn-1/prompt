import subprocess,os,sys,json,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{root}:{root}/scripts');status=[]
(h/'hold_source_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [h/'run_hold.py',h/'PROTOCOL.md',root/'benchmark/isaac_runtime.py',root/'scripts/action_smoothing.py']},indent=2))
for main in [.5,1.5]:
 for aux in [-30,30]:
  name=f'hold_m{main}_a{float(aux)}';print('START',name,flush=True)
  with (h/(name+'.log')).open('w') as f:r=subprocess.run([sys.executable,str(h/'run_hold.py'),'--main',str(main),'--aux',str(aux)],env=env,stdout=f,stderr=subprocess.STDOUT)
  status.append({'case':name,'returncode':r.returncode});(h/'hold_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
  if r.returncode:sys.exit(r.returncode)
