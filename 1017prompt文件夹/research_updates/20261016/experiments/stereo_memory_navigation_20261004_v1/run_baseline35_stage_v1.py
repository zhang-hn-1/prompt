from pathlib import Path
import json,subprocess,sys,os,datetime,hashlib
from bounded_child_cleanup_v1 import run_bounded
from baseline35_conditions_v1 import build_config
H=Path(__file__).resolve().parent;S=H/'baseline35_status_v1.json';assert not S.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
# Protocols for BOTH conditions must predate either run.
for name in ('e0','e2'):
 c=build_config(name);c['stage_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();c['predeclared_utc']=now();(H/f'baseline35_{name}_protocol_v1.json').write_text(json.dumps(c,indent=2)+'\n')
r=dict(status='running',utc=now(),maximum_actual_sequences=2,knownpose_task_prior_baseline=True,strict_visual_admission=False,new_control_candidate=False,sequences=[]);S.write_text(json.dumps(r,indent=2)+'\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_baseline35',TORCH_COMPILE_DISABLE='1')
for name in ('e0','e2'):
 with (H/f'baseline35_{name}_v1.log').open('w') as log:result=run_bounded([sys.executable,str(H/'run_baseline35_navigation_v1.py'),'--condition',name],log,timeout=1200,grace=45,env=env)
 code=124 if result['timeout'] else result['returncode'];D=H/f'baseline35_{name}_run_v1'
 if D.exists():(D/('artifact_manifest_post_close.json' if code==0 else 'artifact_manifest_after_runtime_failure.json')).write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
 r['sequences'].append(dict(condition=name,child_execution=result,normal_close_confirmed=code==0,exit_code=code,end_utc=now()));S.write_text(json.dumps(r,indent=2)+'\n')
 if code!=0:r.update(status='runtime_failed',exit_code=code);S.write_text(json.dumps(r,indent=2)+'\n');sys.exit(code)
r.update(status='finished',exit_code=0,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True)
