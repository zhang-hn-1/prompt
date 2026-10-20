"""ONE frozen-stack initial-condition validation, not another controller."""
from pathlib import Path
import json,hashlib,datetime,os,sys,subprocess,time
from probe_cleanup import seal_existing
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
 p=json.loads((H/'bank_rock_observation_protocol_v1.json').read_text())
 assert all(sha(f)==s for f,s in p['source_sha256'].items())
 base=json.loads((H/'flat_ground_pose_protocol_v1.json').read_text())
 assert all(sha(f)==s for f,s in base['source_sha256'].items())
 assert json.loads((H/'flat_ground_pose_actual_assessment_v1.json').read_text())['decision']=='PASS_CONDITIONAL_ESTIMATED_POSE_TASK_ONLY'
 review=json.loads((H/'bank_rock_observation_author_review_closed_v1.json').read_text())
 assert review['unresolved_critical']==review['unresolved_important']==0
 assert review['driver_sha256']==sha(H/'run_bank_rock_observation_v1.py')
 assert review['stage_sha256']==sha(__file__)
 status=H/'bank_rock_observation_stage_status_v1.json';assert not status.exists()
 r=dict(utc=now(),pid=os.getpid(),stage='RUNNING',maximum_fresh_actual_sequences=1,start_goal_y_offset_m=.15,source_only=True,bank_requested_degrees=10.,navigation_executed=False,formal_triggered=False,whole_research_complete=False)
 def save():status.write_text(json.dumps(r,indent=2)+'\n')
 save()
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_tracking_tube',OPENBLAS_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1')
 with (H/'bank_rock_observation_actual_v1.log').open('x') as f:
  child=subprocess.Popen([sys.executable,str(H/'run_bank_rock_observation_v1.py'),'OBS'],env=env,stdout=f,stderr=subprocess.STDOUT);r['child_pid']=child.pid;save();begin=time.monotonic()
  timed_out=False;forced_kill=False
  try:code=child.wait(timeout=1000)
  except subprocess.TimeoutExpired:
   timed_out=True;child.terminate()
   try:code=child.wait(timeout=30)
   except subprocess.TimeoutExpired:forced_kill=True;child.kill();code=child.wait()
 r.update(stage='WALL_LIMIT_PARTIAL_EVIDENCE_NO_RETRY' if timed_out else ('ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 else 'FAILED_PARTIAL_EVIDENCE_NO_RETRY'),returncode=code,wall_s=time.monotonic()-begin,end_utc=now(),forced_kill=forced_kill,complete=not timed_out and code==0,partial_trace_complete_not_guaranteed=bool(timed_out));save()
 D=H/'bank_rock_observation_run_v1';seal=seal_existing(D,dict(forced_kill=forced_kill,complete=r['complete'],wall_timeout=timed_out));r['post_close_manifest_sha256']=sha(seal);save();assert code==0 and not timed_out

if __name__=='__main__':main()
