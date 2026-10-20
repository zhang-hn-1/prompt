import subprocess,os,sys,json,time,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts');status=[]
(h/'calibration_source_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [h/'roll_pid.py',h/'run_calibration.py',h/'PROTOCOL.md',wm/'benchmark/isaac_runtime.py',wm/'scripts/action_smoothing.py']},indent=2))
for speed in [.5,1.5,-.5,-1.5]:
 name=f'calibration_{speed}';print('START',name,flush=True);t=time.time()
 with (h/(name+'.log')).open('w') as f:r=subprocess.run([sys.executable,str(h/'run_calibration.py'),'--main',str(speed),'--mode','pid'],env=env,stdout=f,stderr=subprocess.STDOUT)
 status.append(dict(case=name,returncode=r.returncode,wall_s=time.time()-t));(h/'calibration_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
 if r.returncode:sys.exit(r.returncode)
