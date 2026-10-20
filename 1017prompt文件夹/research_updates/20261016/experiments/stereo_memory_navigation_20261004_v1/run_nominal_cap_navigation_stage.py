"""Fixed current→history whole runs; never retry scientific arm or tune between."""
from pathlib import Path
import json,os,sys,subprocess,datetime,hashlib
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'nominal_cap_navigation_status.json';assert not status.exists();p=json.loads((H/'nominal_cap_navigation_protocol.json').read_text());row=dict(start_utc=now(),status='running',arms=[],maximum_sequences=1);status.write_text(json.dumps(row,indent=2)+'\n')
for arm in p['arms']:
 entry=dict(arm=arm,status='running',start_utc=now());row['arms'].append(entry);status.write_text(json.dumps(row,indent=2)+'\n')
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_prior_rgb',TORCH_COMPILE_DISABLE='1')
 with (H/'nominal_cap_history_run.log').open('w') as out:r=subprocess.run([sys.executable,str(H/'run_nominal_cap_navigation.py'),arm],stdout=out,stderr=subprocess.STDOUT,env=env)
 entry.update(status='finished',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n')
 folder=H/'nominal_cap_history_run'
 if folder.exists():
  manifest={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in folder.rglob('*') if f.is_file()};(folder/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,scope='post child close; excludes self; notformal navigation'),indent=2)+'\n')
 if r.returncode:
  row.update(status='runtime_failed',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row));sys.exit(r.returncode)
row.update(status='finished',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row))
