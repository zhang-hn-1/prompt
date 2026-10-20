from pathlib import Path
import os, sys, json, hashlib, datetime, subprocess, time, signal
from probe_cleanup import seal_existing
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
    protocol=json.loads((H/'live_async_stereo_navigation_protocol_v1.json').read_text())
    for g in ['source_sha256','input_sha256']:
        for f,s in protocol[g].items():assert sha(f)==s,f
    transport=json.loads((H/'spawn_stereo_transport_verified_v1.json').read_text());assert transport['decision']=='PASS_THREE_ARCHIVED_SPAWN_TRANSPORT_ONLY'
    development=json.loads((H/'paired_texture_surface_development_v1/assessment.json').read_text())
    assert development['decision']=='PASS_DEVELOPMENT_POINT_GATES_AND_RISK_COMPATIBILITY_REDUCTION_ONLY'
    assert all(all(r['checks'].values()) for r in development['sequences']) and all(development['risk_checks'].values())
    review=json.loads((H/'live_async_stereo_navigation_author_review_closed_v1.json').read_text())
    assert review['unresolved_critical']==review['unresolved_important']==0
    assert review['driver_sha256']==sha(H/'run_live_async_stereo_navigation_v1.py')
    status=H/'live_async_stereo_navigation_stage_status_v1.json';assert not status.exists()
    row=dict(start_utc=now(),pid=os.getpid(),stage='RUNNING',maximum_actual_sequences=1,not_original35=True,whole_research_complete=False)
    def save():status.write_text(json.dumps(row,indent=2)+'\n')
    save()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310:/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_partial_surface',TORCH_COMPILE_DISABLE='1')
    with (H/'live_async_stereo_navigation_actual_v1.log').open('x') as log:
        child=subprocess.Popen([sys.executable,str(H/'run_live_async_stereo_navigation_v1.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        row['child_pid']=child.pid;save();begin=time.monotonic();timeout=False;forced=False
        try:code=child.wait(timeout=1800)
        except subprocess.TimeoutExpired:
            timeout=True;os.killpg(child.pid,signal.SIGTERM)
            try:code=child.wait(timeout=30)
            except subprocess.TimeoutExpired:forced=True
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            code=child.wait()
    row.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 and not timeout else 'FAILED_NO_RETRY',end_utc=now(),exit_code=code,wall_s=time.monotonic()-begin,wall_timeout=timeout,forced_kill=forced)
    out=H/'live_async_stereo_navigation_run_v1'
    if out.exists():row['post_close_manifest_sha256']=sha(seal_existing(out,dict(complete=code==0 and not timeout,forced_kill=forced,wall_timeout=timeout)))
    save();print(json.dumps(row,indent=2));raise SystemExit(code if not timeout else 1)
if __name__=='__main__':main()
