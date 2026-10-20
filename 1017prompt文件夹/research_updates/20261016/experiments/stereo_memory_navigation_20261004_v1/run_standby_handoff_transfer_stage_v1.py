from pathlib import Path
import json,sys,os,datetime,hashlib
from bounded_child_cleanup_v1 import run_bounded
H=Path(__file__).resolve().parent;S=H/'standby_handoff_transfer_status_v1.json';assert not S.exists();gate=json.loads((H/'standby_handoff_e0_run_v1/standby_handoff_assessment_v1.json').read_text());assert gate['local_handoff_candidate_accepted'];now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();r=dict(status='running',utc=now(),maximum_actual_sequences=2,knownpose_task_prior=True,strict_visual_admission=False,same_single_control_candidate=True,sequences=[]);S.write_text(json.dumps(r,indent=2)+'\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_baseline35',TORCH_COMPILE_DISABLE='1')
for name in ('e2','e1'):
 with (H/f'standby_handoff_{name}_transfer_v1.log').open('w') as log:result=run_bounded([sys.executable,str(H/'run_standby_handoff_transfer_v1.py'),'--condition',name],log,timeout=1200,grace=45,env=env)
 code=124 if result['timeout'] else result['returncode'];D=H/f'standby_handoff_{name}_transfer_run_v1'
 if D.exists():(D/('artifact_manifest_post_close.json' if code==0 else 'artifact_manifest_after_runtime_failure.json')).write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
 r['sequences'].append(dict(condition=name,child_execution=result,normal_close_confirmed=code==0,exit_code=code,end_utc=now()));S.write_text(json.dumps(r,indent=2)+'\n')
 if code!=0:r.update(status='runtime_failed',exit_code=code);S.write_text(json.dumps(r,indent=2)+'\n');sys.exit(code)
r.update(status='finished',exit_code=0,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True)
