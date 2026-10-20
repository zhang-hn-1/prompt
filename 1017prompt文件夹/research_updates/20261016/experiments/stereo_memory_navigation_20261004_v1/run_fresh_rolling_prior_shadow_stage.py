from pathlib import Path
import json,os,sys,subprocess,datetime,hashlib
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'fresh_rolling_prior_shadow_status.json';assert not status.exists();row=dict(start_utc=now(),status='running',maximum_sequences=1,arm='current',shadow='same-source history readonly');status.write_text(json.dumps(row,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_fresh_rolling_prior_shadow',TORCH_COMPILE_DISABLE='1')
with (H/'fresh_rolling_prior_shadow_run.log').open('w') as out:r=subprocess.run([sys.executable,str(H/'run_fresh_rolling_prior_shadow.py'),'current'],stdout=out,stderr=subprocess.STDOUT,env=env)
row.update(status='finished' if not r.returncode else 'runtime_failed',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');folder=H/'fresh_rolling_prior_shadow_current_run'
if folder.exists():
 manifest={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in folder.rglob('*') if f.is_file()};(folder/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,scope='post childclose; self excluded; risk knownpose diagnostic'),indent=2)+'\n')
print(json.dumps(row));sys.exit(r.returncode)
