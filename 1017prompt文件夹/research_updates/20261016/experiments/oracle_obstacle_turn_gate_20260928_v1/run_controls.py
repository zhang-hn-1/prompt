import os,sys,subprocess,json,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{root}:{root}/scripts')
paths=[h/'run_ungated.py',h/'CONTACT_CONTROL_PROTOCOL.md']+list((h/'scenes').glob('*'))
(h/'contact_control_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},indent=2))
status=[]
for kind in ['LR','RL']:
 name=kind+'_blocked_ungated';print('START',name,flush=True)
 with (h/(name+'.log')).open('w') as log:r=subprocess.run([sys.executable,str(h/'run_ungated.py'),'--kind',kind,'--mode','blocked'],env=env,stdout=log,stderr=subprocess.STDOUT)
 status.append({'name':name,'returncode':r.returncode});(h/'control_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
 if r.returncode:sys.exit(r.returncode)
