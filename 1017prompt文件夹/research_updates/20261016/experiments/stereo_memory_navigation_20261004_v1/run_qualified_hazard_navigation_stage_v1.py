"""One bounded actual consumer run, no scientific retry or baseline repetition."""
from pathlib import Path
import datetime, hashlib, json, os, subprocess, sys
H=Path(__file__).resolve().parent
status=H/'qualified_hazard_navigation_status_v1.json'
assert not status.exists()
assert '477 passed' in (H/'qualified_hazard_full_CPU_v1.log').read_text()
assessment=json.loads((H/'qualified_hazard_development_replay_v1/assessment_v1.json').read_text())
assert assessment['appearance_precision_ge95_all_nonempty']
protocol=json.loads((H/'qualified_hazard_navigation_protocol_v1.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert all(sha(f)==v for f,v in protocol['source_sha256'].items())
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
row=dict(status='running',start_utc=now(),maximum_sequences=1,
         baseline_reused_not_rerun=True,estimated_pose_not_admitted=True,
         whole_research_complete=False,formal_navigation=False)
status.write_text(json.dumps(row,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',
         PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',
         MPLCONFIGDIR='/tmp/mpl_qualified_hazard',TORCH_COMPILE_DISABLE='1')
with (H/'qualified_hazard_navigation_run_v1.log').open('w') as log:
    r=subprocess.run([sys.executable,str(H/'run_qualified_hazard_navigation_v1.py'),'observe'],
                     stdout=log,stderr=subprocess.STDOUT,env=env)
folder=H/'qualified_hazard_navigation_run_v1'
if folder.exists():
    manifest={str(f):sha(f) for f in folder.rglob('*') if f.is_file()}
    (folder/'artifact_manifest_post_close.json').write_text(json.dumps(dict(
        utc=now(),artifact_sha256=manifest,scope='after child close; excludes self; not formal navigation'),indent=2)+'\n')
row.update(status='finished' if r.returncode==0 else 'runtime_failed',exit_code=r.returncode,end_utc=now())
status.write_text(json.dumps(row,indent=2)+'\n')
print(json.dumps(row,indent=2));sys.exit(r.returncode)
