from pathlib import Path
import json,os,sys,subprocess,datetime,hashlib
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'dynamic_mask_probe_status.json';assert not status.exists();row=dict(start_utc=now(),status='running',maximum_sequences=1,episode_budget_s=12.,validation_frames=[0,30,60,120],model_inferences=0,diagnostic_only=True);status.write_text(json.dumps(row,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_dynamic_mask_probe',TORCH_COMPILE_DISABLE='1')
with (H/'dynamic_mask_probe_run.log').open('w') as out:r=subprocess.run([sys.executable,str(H/'run_dynamic_mask_probe.py'),'current'],stdout=out,stderr=subprocess.STDOUT,env=env)
row.update(status='finished' if not r.returncode else 'runtime_failed',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');folder=H/'dynamic_mask_probe_run'
if folder.exists():
 m={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in folder.rglob('*') if f.is_file()};(folder/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=m,scope='postclose no self; 12s ownmask diagnostic, notnavigation'),indent=2)+'\n')
print(json.dumps(row));sys.exit(r.returncode)
