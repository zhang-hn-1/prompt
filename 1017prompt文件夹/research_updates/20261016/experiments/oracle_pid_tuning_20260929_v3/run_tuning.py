import subprocess,os,sys,json,hashlib,time
from pathlib import Path
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
identity={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [h/'PROTOCOL.md',h/'run_probe.py',h/'roll_pid.py',h/'bank_mapping.py',h/'bank_mapping.json']};(h/'tuning_identity.json').write_text(json.dumps(identity,indent=2))
status=[]
settings=[('D070','.8,.1,.70'),('D105','.8,.1,1.05'),('P040D070','.4,.1,.70'),('P120D070','1.2,.1,.70')]
for tag,gains in settings+[('supervisor060','.8,.1,.35')]:
 for main in ([1.] if tag=='supervisor060' else [1.,-1.]):
  cmd=[sys.executable,str(h/'run_probe.py'),'--main',str(main),'--mode','pid','--tag',tag,'--gains',gains,'--limit',str(.6 if tag=='supervisor060' else .3)];name=f'{tag}_{main}';print('START',name,flush=True);t=time.time()
  with (h/(name+'.log')).open('w') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
  status.append(dict(case=name,returncode=r.returncode,wall_s=time.time()-t));(h/'tuning_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
  if r.returncode:sys.exit(r.returncode)
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items())
