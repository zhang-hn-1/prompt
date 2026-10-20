"""Two predeclared complete6s validation acquisitions, NO fit or model actions."""
from pathlib import Path
import os,json,hashlib,datetime,subprocess,sys,time
H=Path(__file__).resolve().parent;S=H/'hypothesis_learning_acquisition_status_v1.json';assert not S.exists();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();p=H/'hypothesis_learning_acquisition_runtime_protocol_v1.json';config=json.loads(p.read_text());assert all(sha(Path(f))==s for f,s in config['source_sha256'].items());review=H/'hypothesis_learning_acquisition_author_review_v1.json';rview=json.loads(review.read_text());assert rview['critical']==0 and rview['important']==0;assert rview['driver_sha256']==sha(H/'run_hypothesis_learning_acquisition_v1.py');assert rview['protocol_sha256']==sha(p)
r=dict(utc=now(),parent_pid=os.getpid(),stage='RUNNING',maximum_whole_source_sequences=2,fit_started=False,model_actions=False,protocol_sha256=sha(p),closed=[])
def save():S.write_text(json.dumps(r,indent=2)+'\n')
save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_hypothesis_learning',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1')
for case in config['fresh_cases']:
 r.update(case=case,stage='RUNNING');save()
 with (H/(case+'_actual_v1.log')).open('x') as log:
  child=subprocess.Popen([sys.executable,str(H/'run_hypothesis_learning_acquisition_v1.py'),'D3',case],stdout=log,stderr=subprocess.STDOUT,env=env);r['child_pid']=child.pid;save();begin=time.monotonic()
  try:code=child.wait(timeout=900)
  except subprocess.TimeoutExpired:
   child.terminate()
   try:child.wait(timeout=30)
   except subprocess.TimeoutExpired:child.kill();child.wait()
   code=124
 D=H/(case+'_run_v1');manifest={str(f):sha(f) for f in D.rglob('*') if f.is_file()} if D.exists() else {};seal=D/'artifact_manifest_post_close.json'
 if D.exists():seal.write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,role='complete6s SOURCE-only fresh validation before fit',no_navigation_success_claim=True),indent=2)+'\n')
 entry=dict(case=case,code=code,wall_s=time.monotonic()-begin,closed_utc=now(),manifest_sha256=sha(seal) if seal.exists() else None);r['closed'].append(entry);r['child_pid']=None;save()
 if code!=0:r['stage']='FAILED_PRESERVED_NO_RETRY';save();sys.exit(code)
r.update(stage='TWO_WHOLE_SEQUENCES_SEALED_BEFORE_FIT',end_utc=now(),fit_started=False);save();print(json.dumps(r),flush=True)
