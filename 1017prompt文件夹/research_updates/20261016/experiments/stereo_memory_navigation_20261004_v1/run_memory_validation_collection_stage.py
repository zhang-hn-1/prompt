from pathlib import Path
import json,sys,subprocess,datetime,os,hashlib
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'memory_validation_collection_status.json';assert not status.exists();row=dict(start_utc=now(),status='running',maximum_sequences=2,sequences=[],purpose='seal two entire validation sequences before fitting; do not read heldout labels/scores');status.write_text(json.dumps(row,indent=2)+'\n')
for kind in ['far','near']:
 entry=dict(kind=kind,status='running',start_utc=now());row['sequences'].append(entry);status.write_text(json.dumps(row,indent=2)+'\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_prior_rgb',TORCH_COMPILE_DISABLE='1')
 with (H/('memory_validation_'+kind+'_run.log')).open('w') as out:r=subprocess.run([sys.executable,str(H/'run_memory_validation_collection.py'),kind],stdout=out,stderr=subprocess.STDOUT,env=env)
 entry.update(status='finished' if r.returncode==0 else 'runtime_failed',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');o=H/('memory_validation_'+kind+'_run')
 if o.exists():
  manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in o.rglob('*') if p.is_file()};(o/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,whole_sequence_heldout=True,label_deserialization_by_fitter_forbidden_before_fit_and_prediction_seal=True),indent=2)+'\n')
 if r.returncode:
  row.update(status='runtime_failed',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');sys.exit(r.returncode)
row.update(status='finished_both_validation_sequences_sealed',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row))
