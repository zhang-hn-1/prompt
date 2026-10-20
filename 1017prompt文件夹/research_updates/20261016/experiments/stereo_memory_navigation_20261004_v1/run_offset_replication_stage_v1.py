"""ONE frozen-stack initial-condition validation, not another controller."""
from pathlib import Path
import json,hashlib,datetime,os,sys,subprocess,time
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
 p=json.loads((H/'offset_replication_protocol_v1.json').read_text())
 assert all(sha(f)==s for f,s in p['source_sha256'].items())
 base=json.loads((H/'flat_ground_pose_protocol_v1.json').read_text())
 assert all(sha(f)==s for f,s in base['source_sha256'].items())
 assert json.loads((H/'flat_ground_pose_actual_assessment_v1.json').read_text())['decision']=='PASS_CONDITIONAL_ESTIMATED_POSE_TASK_ONLY'
 review=json.loads((H/'offset_replication_author_review_closed_v1.json').read_text())
 assert review['unresolved_critical']==review['unresolved_important']==0
 assert review['driver_sha256']==sha(H/'run_offset_replication_D4_v1.py')
 assert review['stage_sha256']==sha(__file__)
 status=H/'offset_replication_stage_status_v1.json';assert not status.exists()
 r=dict(utc=now(),pid=os.getpid(),stage='RUNNING',maximum_fresh_actual_sequences=1,start_goal_y_offset_m=.15,frozen_D3_controller=True,formal_triggered=False,whole_research_complete=False)
 def save():status.write_text(json.dumps(r,indent=2)+'\n')
 save()
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_tracking_tube',OPENBLAS_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1')
 with (H/'offset_replication_actual_v1.log').open('x') as f:
  child=subprocess.Popen([sys.executable,str(H/'run_offset_replication_D4_v1.py'),'D4'],env=env,stdout=f,stderr=subprocess.STDOUT);r['child_pid']=child.pid;save();begin=time.monotonic()
  try:code=child.wait(timeout=1500)
  except subprocess.TimeoutExpired:
   child.terminate()
   try:child.wait(timeout=30)
   except subprocess.TimeoutExpired:child.kill();child.wait()
   r.update(stage='WALL_LIMIT_PRESERVED_NO_RETRY',end_utc=now());save();raise
 r.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 else 'FAILED_PRESERVED_NO_RETRY',returncode=code,wall_s=time.monotonic()-begin,end_utc=now());save()
 D=H/'offset_replication_D4_run_v1';manifest={str(f):sha(f) for f in D.rglob('*') if f.is_file()};seal=D/'artifact_manifest_post_close.json';seal.write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,whole_research_complete=False,formal_triggered=False),indent=2)+'\n');r['post_close_manifest_sha256']=sha(seal);save();assert code==0
if __name__=='__main__':main()
