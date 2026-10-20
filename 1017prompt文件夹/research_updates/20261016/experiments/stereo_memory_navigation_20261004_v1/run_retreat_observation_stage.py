from pathlib import Path
import sys,json,datetime,os,subprocess,hashlib
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'retreat_observation_status.json';assert not status.exists();row=dict(start_utc=now(),status='running',maximum_sequences=1,diagnostic_duration_s=18.,not_navigation=True);status.write_text(json.dumps(row,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_prior_rgb',TORCH_COMPILE_DISABLE='1')
with (H/'retreat_observation_run.log').open('w') as out:r=subprocess.run([sys.executable,str(H/'run_retreat_observation_probe.py'),'current'],stdout=out,stderr=subprocess.STDOUT,env=env)
row.update(status='finished' if r.returncode==0 else 'runtime_failed',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n')
o=H/'retreat_observation_run'
if o.exists():
 m={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in o.rglob('*') if p.is_file()};(o/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=m,scope='post child shutdown; excludes self'),indent=2)+'\n')
print(json.dumps(row));sys.exit(r.returncode)
