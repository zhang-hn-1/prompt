import os,sys,json,datetime,subprocess,hashlib
from pathlib import Path
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
assert not (H/'guarded_reference_status.json').exists()
row=dict(start_utc=now(),status='running',maximum_sequences=1)
(H/'guarded_reference_status.json').write_text(json.dumps(row,indent=2)+'\n')
with (H/'guarded_reference_run.log').open('w') as f:
 r=subprocess.run([sys.executable,str(H/'run_guarded_reference.py')],stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
row.update(end_utc=now(),status='finished',exit_code=r.returncode)
(H/'guarded_reference_status.json').write_text(json.dumps(row,indent=2)+'\n')
# Parent remains alive after SDK close: include close-generated feedback without changing pre-close evidence.
out=H/'guarded_reference_run'
if r.returncode==0:
 manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file()}
 (out/'artifact_manifest_post_close.json').write_text(json.dumps(dict(saved_utc=now(),artifact_sha256=manifest,scope='all files after child exit, excluding this manifest itself'),indent=2)+'\n')
print(json.dumps(row,indent=2));raise SystemExit(r.returncode)
