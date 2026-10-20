"""One actual same-version B0/C1 pair; each900s wall, no scientific retry."""
from pathlib import Path
import datetime,json,hashlib,os,subprocess,sys,time
H=Path(__file__).resolve().parent;status=H/'source_event_handoff_stage_status_v1.json';assert not status.exists();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();p=json.loads((H/'source_event_handoff_protocol_v1.json').read_text());assert all(sha(f)==v for f,v in p['source_sha256'].items());assert '544 passed' in (H/'source_event_handoff_full_suite_v1.log').read_text();review=json.loads((H/'source_event_handoff_author_review_closed_v1.json').read_text());assert review['unresolved_critical']==0 and review['unresolved_important']==0;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();row=dict(stage='RUN_ACTUAL_SAME_VERSION_B0_C1',start_utc=now(),pid=os.getpid(),maximum_actual_sequences=2,per_sequence_max_wall_s=900,formal_triggered=False,whole_research_complete=False,arms=[])
def save():status.write_text(json.dumps(row,indent=2)+'\n')
save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_source_event_handoff',TORCH_COMPILE_DISABLE='1')
for arm in ['B0','C1']:
 sub=dict(arm=arm,start_utc=now(),stage='RUNNING',wall_begin=time.monotonic());row['arms'].append(sub);save()
 with (H/f'source_event_handoff_{arm}_actual_v1.log').open('x') as log:
  child=subprocess.Popen([sys.executable,str(H/'run_source_event_handoff_v1.py'),arm],env=env,stdout=log,stderr=subprocess.STDOUT);sub['child_pid']=child.pid;save()
  try:code=child.wait(timeout=900)
  except subprocess.TimeoutExpired:
   child.terminate()
   try:child.wait(timeout=30)
   except subprocess.TimeoutExpired:child.kill();child.wait()
   sub.update(stage='WALL_TIMEOUT_PRESERVED',end_utc=now());row['stage']='FAILED_PRESERVED_STOP_NO_RETRY';save();raise
 sub.update(stage='ACTUAL_CLOSED' if code==0 else 'FAILED_PRESERVED',returncode=code,end_utc=now(),wall_s=time.monotonic()-sub['wall_begin']);row['stage']='RUN_ACTUAL_SAME_VERSION_B0_C1' if code==0 else 'FAILED_PRESERVED_STOP_NO_RETRY';save();assert code==0,'preserve failing arm, no scientific retries/no candidate start after B0 failure'
 out=H/f'source_event_handoff_{arm}_run_v1';manifest={str(f):sha(f) for f in out.rglob('*') if f.is_file()};(out/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,arm=arm,knownpose_privileged=True,formal_triggered=False,whole_research_complete=False),indent=2)+'\n');sub['post_close_manifest_sha256']=sha(out/'artifact_manifest_post_close.json');save()
row.update(stage='BOTH_ACTUAL_ARMS_CLOSED_NEEDS_VERIFICATION',end_utc=now());save()
