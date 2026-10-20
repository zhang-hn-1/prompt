from pathlib import Path
import json,subprocess,sys,os,datetime,hashlib
H=Path(__file__).resolve().parent;S=H/'complete_exact_reference_status_v1.json';assert not S.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();status=dict(start_utc=now(),status='running',maximum_sequences=2,variants=[]);S.write_text(json.dumps(status,indent=2)+'\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_complete_reference')
for arm in ['B0','exact']:
 row=dict(variant=arm,start_utc=now(),status='running');status['variants'].append(row);S.write_text(json.dumps(status,indent=2)+'\n')
 with (H/f'complete_exact_reference_{arm}_v1.log').open('w') as log:
  try:r=subprocess.run([sys.executable,str(H/'run_complete_exact_reference_v1.py'),arm],stdout=log,stderr=subprocess.STDOUT,env=env,timeout=1200);code=r.returncode
  except subprocess.TimeoutExpired:code=124
 row.update(exit_code=code,end_utc=now(),status='finished' if code==0 else 'runtime_failed');D=H/f'complete_exact_reference_{arm}_run_v1'
 if D.exists():(D/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),files={str(f.relative_to(D)):sha(f) for f in D.rglob('*') if f.is_file()}),indent=2)+'\n')
 S.write_text(json.dumps(status,indent=2)+'\n')
 if code:status.update(status='runtime_failed',end_utc=now());S.write_text(json.dumps(status,indent=2)+'\n');sys.exit(code)
status.update(status='finished',end_utc=now());S.write_text(json.dumps(status,indent=2)+'\n');print(json.dumps(status),flush=True)
