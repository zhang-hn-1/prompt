import subprocess,os,sys,json,hashlib,time
from pathlib import Path
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
selection=json.loads((h/'selected_candidate.json').read_text());assert selection['tuning_eligible'];g=selection['gains'];gains=f"{g['kp']},{g['ki']},{g['kd']}"
(h/'fresh_frozen_identity.json').write_text(json.dumps({str(h/n):hashlib.sha256((h/n).read_bytes()).hexdigest() for n in ['selected_candidate.json','roll_pid.py','bank_mapping.py','bank_mapping.json','run_probe.py']},indent=2))
status=[]
for main in [.5,-1.5]:
 for tag,gain in [('fresh_base','.8,.1,.35'),('fresh_selected',gains)]:
  name=f'{tag}_{main}';cmd=[sys.executable,str(h/'run_probe.py'),'--main',str(main),'--mode','pid','--tag',tag,'--gains',gain,'--fresh'];print('START',name,flush=True);t=time.time()
  with (h/(name+'.log')).open('w') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
  status.append(dict(case=name,returncode=r.returncode,wall_s=time.time()-t));(h/'fresh_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
  if r.returncode:sys.exit(r.returncode)
