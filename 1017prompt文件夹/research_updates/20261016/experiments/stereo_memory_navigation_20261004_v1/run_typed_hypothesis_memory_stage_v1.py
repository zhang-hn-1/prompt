"""Two bounded actual arms; no C1 if shared bootstrap cannot exercise memory."""
from pathlib import Path
import datetime,json,hashlib,os,subprocess,sys,time
H=Path(__file__).resolve().parent;status=H/'typed_hypothesis_memory_stage_status_v1.json';assert not status.exists();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();p=json.loads((H/'typed_hypothesis_memory_protocol_v1.json').read_text());assert all(sha(f)==v for f,v in p['source_sha256'].items());assert json.loads((H/'partial_set_clock_repair_assessment_v2.json').read_text())['decision']=='PASS_LOCAL_SOURCE_HYPOTHESIS_PREREQUISITES_ONLY';review=json.loads((H/'typed_hypothesis_memory_author_review_closed_v1.json').read_text());suite=json.loads((H/'typed_hypothesis_memory_suite_evidence_v1.json').read_text());assert suite['passed'] and suite['returncode']==0 and all(sha(f)==v for f,v in suite['test_and_implementation_sha256'].items());assert sha(suite['full_suite_log'])==suite['full_suite_log_sha256'];assert review['unresolved_critical']==0 and review['unresolved_important']==0;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();row=dict(stage='RUN_ACTUAL_CURRENT_VS_STATIC_SEED',start_utc=now(),pid=os.getpid(),maximum_actual_sequences=2,per_sequence_max_wall_s=1500,formal_triggered=False,whole_research_complete=False,arms=[])
def save():status.write_text(json.dumps(row,indent=2)+'\n')
save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_typed_hypothesis_memory',TORCH_COMPILE_DISABLE='1')
for arm in ['B0','C1']:
 sub=dict(arm=arm,start_utc=now(),stage='RUNNING',wall_begin=time.monotonic());row['arms'].append(sub);save()
 with (H/f'typed_hypothesis_memory_{arm}_actual_v1.log').open('x') as log:
  child=subprocess.Popen([sys.executable,str(H/'run_typed_hypothesis_memory_v1.py'),arm],env=env,stdout=log,stderr=subprocess.STDOUT);sub['child_pid']=child.pid;save()
  try:code=child.wait(timeout=1500)
  except subprocess.TimeoutExpired:
   child.terminate()
   try:child.wait(timeout=30)
   except subprocess.TimeoutExpired:child.kill();child.wait()
   sub.update(stage='WALL_TIMEOUT_PRESERVED',end_utc=now());row['stage']='FAILED_PRESERVED_STOP_NO_RETRY';save();raise
 sub.update(stage='ACTUAL_CLOSED' if code==0 else 'FAILED_PRESERVED',returncode=code,end_utc=now(),wall_s=time.monotonic()-sub['wall_begin']);row['stage']='RUN_ACTUAL_CURRENT_VS_STATIC_SEED' if code==0 else 'FAILED_PRESERVED_STOP_NO_RETRY';save()
 out=H/f'typed_hypothesis_memory_{arm}_run_v1';manifest={str(f):sha(f) for f in out.rglob('*') if f.is_file()};(out/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,arm=arm,knownpose_privileged=True,formal_triggered=False,whole_research_complete=False),indent=2)+'\n');sub['post_close_manifest_sha256']=sha(out/'artifact_manifest_post_close.json');save()
 ident=json.loads((out/'identity.json').read_text())
 if arm=='B0' and (ident['bootstrap_blocked'] or ident['observation_handoff_time_s'] is None):
  row.update(stage='BASELINE_BOOTSTRAP_NOT_EXERCISED_C1_NOT_LAUNCHED',end_utc=now());save();sys.exit(0)
 if code!=0:raise RuntimeError('preserved actual runtime failure; no retry')
row.update(stage='BOTH_ACTUAL_ARMS_CLOSED_NEEDS_VERIFICATION',end_utc=now());save()
