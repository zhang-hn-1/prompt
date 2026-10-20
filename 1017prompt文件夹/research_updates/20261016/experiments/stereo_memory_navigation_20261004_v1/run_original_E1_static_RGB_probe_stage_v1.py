from pathlib import Path
import os,sys,json,hashlib,datetime,subprocess,signal,time
from probe_cleanup import seal_existing
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
 p=json.loads((H/'original_E1_static_RGB_probe_protocol_v1.json').read_text())
 for g in ['source_sha256','input_sha256']:
  for f,s in p[g].items():assert sha(f)==s,f
 status=H/'original_E1_static_RGB_probe_status_v1.json';assert not status.exists();row=dict(stage='RUNNING',start_utc=now(),pid=os.getpid(),maximum_actual_sequences=1,maximum_render_snapshots=24,no_NN=True,no_requests=True,whole_research_complete=False)
 def save():status.write_text(json.dumps(row,indent=2)+'\n')
 save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/ball_nav_mpl',TORCH_COMPILE_DISABLE='1')
 with (H/'original_E1_static_RGB_probe_actual_v1.log').open('x') as log:
  child=subprocess.Popen([sys.executable,str(H/'run_original_E1_static_RGB_probe_v1.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);row['child_pid']=child.pid;save();begin=time.monotonic();timeout=False;forced=False
  try:code=child.wait(timeout=600)
  except subprocess.TimeoutExpired:
   timeout=True;os.killpg(child.pid,signal.SIGTERM)
   try:code=child.wait(timeout=30)
   except subprocess.TimeoutExpired:forced=True;os.killpg(child.pid,signal.SIGKILL);code=child.wait()
 row.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 and not timeout else 'FAILED_NO_RETRY',end_utc=now(),exit_code=code,wall_s=time.monotonic()-begin,wall_timeout=timeout,forced_kill=forced)
 out=H/'original_E1_static_RGB_probe_run_v1'
 if out.exists():row['post_close_manifest_sha256']=sha(seal_existing(out,dict(complete=code==0 and not timeout,wall_timeout=timeout,forced_kill=forced)))
 save();print(json.dumps(row,indent=2));raise SystemExit(code if not timeout else 1)
if __name__=='__main__':main()
