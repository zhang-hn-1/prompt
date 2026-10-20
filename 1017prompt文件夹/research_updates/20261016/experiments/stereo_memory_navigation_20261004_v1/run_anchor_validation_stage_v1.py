from pathlib import Path
import json,subprocess,sys,os,datetime,hashlib
H=Path(__file__).resolve().parent;status=H/'anchor_validation_status_v1.json';assert not status.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();row=dict(status='running',start_utc=now(),max_sequences=1,purpose='one frozen source filter whole-sequence validation, not navigation');status.write_text(json.dumps(row,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_prior_rgb',TORCH_COMPILE_DISABLE='1')
with (H/'anchor_validation_fresh_run_v1.log').open('w') as out:r=subprocess.run([sys.executable,str(H/'run_anchor_validation_fresh_v1.py'),'near'],stdout=out,stderr=subprocess.STDOUT,env=env)
row.update(status='finished' if r.returncode==0 else 'runtime_failed',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');o=H/'anchor_validation_fresh_run_v1'
if o.exists():
 files={str(p):sha(p) for p in o.rglob('*') if p.is_file()};(o/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=files,predictions_sealed_before_label_assessment=True,whole_sequence=True,not_fresh_scene=True),indent=2)+'\n')
print(json.dumps(row),flush=True);sys.exit(r.returncode)
