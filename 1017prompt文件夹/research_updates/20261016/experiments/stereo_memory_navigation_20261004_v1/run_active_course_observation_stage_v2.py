"""ONE actual35s sensor probe, no scientific retries; preserve failed evidence."""
from pathlib import Path
import datetime,hashlib,json,os,subprocess,sys,time
H=Path(__file__).resolve().parent
status=H/'active_course_observation_status_v1.json'
assert not status.exists()
assert '516 passed' in (H/'active_course_observation_full_suite_v1.log').read_text()
review=json.loads((H/'active_course_observation_author_review_closed_v1.json').read_text())
assert review['first_review_closed'] and review['unresolved_critical']==0 and review['unresolved_important']==0
protocol=json.loads((H/'active_course_observation_protocol_v2.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert all(sha(p)==v for p,v in protocol['source_sha256'].items())
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
row=dict(stage='running_actual_one_sequence',start_utc=now(),pid=os.getpid(),maximum_sequences=1,
         no_scientific_retry=True,formal_navigation=False,whole_research_complete=False)
status.write_text(json.dumps(row,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',
         PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',
         MPLCONFIGDIR='/tmp/mpl_active_course_observation',TORCH_COMPILE_DISABLE='1')
begin=time.perf_counter()
with (H/'active_course_observation_actual_run_v1.log').open('w') as log:
    child=subprocess.Popen([sys.executable,str(H/'run_active_course_observation_v2.py'),'observe'],stdout=log,stderr=subprocess.STDOUT,env=env)
    row['child_pid']=child.pid;status.write_text(json.dumps(row,indent=2)+'\n')
    try:
        code=child.wait(timeout=900)
        row.update(stage='finished_actual_probe_only' if code==0 else 'runtime_failed_preserved',exit_code=code)
    except subprocess.TimeoutExpired:
        child.terminate()
        try:child.wait(timeout=30)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        row.update(stage='bounded_walltime_exceeded_preserved',exit_code=child.returncode)
folder=H/'active_course_observation_run_v1'
if folder.exists():
    (folder/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),scope='after child close, excludes self',artifact_sha256={str(p):sha(p) for p in folder.rglob('*') if p.is_file()},no_source_or_navigation_admission=True),indent=2)+'\n')
row.update(end_utc=now(),elapsed_wall_s=time.perf_counter()-begin,assessment_pending=True)
status.write_text(json.dumps(row,indent=2)+'\n')
print(json.dumps(row,indent=2));sys.exit(0 if row.get('exit_code')==0 else 1)
