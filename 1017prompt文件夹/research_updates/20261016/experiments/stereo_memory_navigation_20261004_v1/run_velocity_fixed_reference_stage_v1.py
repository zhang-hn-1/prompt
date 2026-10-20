from pathlib import Path
import subprocess,sys,json,os,datetime,hashlib
H=Path(__file__).resolve().parent;status=H/'velocity_fixed_reference_status_v1.json';assert not status.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();p=json.loads((H/'velocity_fixed_reference_protocol_v1.json').read_text());assert all(sha(f)==s for f,s in p['source_sha256'].items());row=dict(status='running',start_utc=now(),maximum_sequences=2,sequences=[],fixed_local_tracking_not_navigation=True);status.write_text(json.dumps(row,indent=2)+'\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_prior_rgb',TORCH_COMPILE_DISABLE='1')
for variant in ['B0','velocity']:
 e=dict(variant=variant,start_utc=now(),status='running');row['sequences'].append(e);status.write_text(json.dumps(row,indent=2)+'\n')
 with (H/('velocity_fixed_reference_'+variant+'_v1.log')).open('w') as out:r=subprocess.run([sys.executable,str(H/'run_velocity_fixed_reference_v1.py'),variant],stdout=out,stderr=subprocess.STDOUT,env=env)
 e.update(exit_code=r.returncode,status='finished' if r.returncode==0 else 'runtime_failed',end_utc=now());o=H/('velocity_fixed_reference_'+variant+'_run_v1')
 if o.exists():
  (o/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256={str(f):sha(f) for f in o.rglob('*') if f.is_file()},fixed_local_tracking_not_navigation=True),indent=2)+'\n')
 status.write_text(json.dumps(row,indent=2)+'\n')
 if r.returncode:row.update(status='runtime_failed',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');sys.exit(r.returncode)
row.update(status='finished',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row),flush=True)
