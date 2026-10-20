"""Two remaining fixed-baseline conditions; preserves E2 navigation rejection."""
from pathlib import Path
import json,sys,os,datetime,hashlib
from bounded_child_cleanup_v1 import run_bounded
H=Path(__file__).resolve().parent;S=H/'action_video_fresh_status_v1.json'
assert not S.exists()
assert json.loads((H/'native_pose_baseline35_e0_status_v1.json').read_text())['status']=='finished'
r0=json.loads((H/'native_pose_baseline35_e0_run_v1/native_pose_assessment_v2.json').read_text());assert r0['estimated_pose_used_online_at_registration_and_navigation'] and r0['captures']==351
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
r=dict(status='running',start_utc=now(),maximum_actual_sequences=2,role='two fresh complete video-forecast validation acquisitions, no model fit; labels whole-sealed first',prior_knownpose_navigation_rejections_preserved=True,sequences=[])
S.write_text(json.dumps(r,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_baseline35',TORCH_COMPILE_DISABLE='1')
for name in ('e0','e1'):
    with (H/f'action_video_fresh_{name}_v1.log').open('w') as log:
        result=run_bounded([sys.executable,str(H/'run_action_video_fresh_acquisitions_v1.py'),'--condition',name],log,timeout=1200,grace=45,env=env)
    code=124 if result['timeout'] else result['returncode'];D=H/f'action_video_fresh_{name}_run_v1'
    if D.exists():
        (D/('artifact_manifest_post_close.json' if code==0 else 'artifact_manifest_after_runtime_failure.json')).write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
    r['sequences'].append(dict(condition=name,child_execution=result,normal_close_confirmed=code==0,exit_code=code,end_utc=now()));S.write_text(json.dumps(r,indent=2)+'\n')
    if code!=0:
        r.update(status='runtime_failed',exit_code=code);S.write_text(json.dumps(r,indent=2)+'\n');sys.exit(code)
r.update(status='finished',exit_code=0,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True)
