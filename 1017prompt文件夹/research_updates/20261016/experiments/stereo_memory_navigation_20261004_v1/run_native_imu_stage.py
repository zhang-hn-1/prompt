import os,sys,json,datetime,subprocess,hashlib
from pathlib import Path
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();assert not (H/'native_imu_status.json').exists()
row=dict(start_utc=now(),status='running',maximum_sequences=1);(H/'native_imu_status.json').write_text(json.dumps(row,indent=2)+'\n')
with (H/'native_imu_run.log').open('w') as f:
 r=subprocess.run([sys.executable,str(H/'run_native_imu.py')],stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
row.update(end_utc=now(),status='finished',exit_code=r.returncode);(H/'native_imu_status.json').write_text(json.dumps(row,indent=2)+'\n');out=H/'native_imu_run'
if r.returncode==0:
 manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file()};(out/'artifact_manifest_post_close.json').write_text(json.dumps(dict(saved_utc=now(),artifact_sha256=manifest,scope='post child exit including feedback; excludes self'),indent=2)+'\n')
print(json.dumps(row,indent=2));raise SystemExit(r.returncode)
