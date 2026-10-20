"""Post-close independent factual checks; never launches another simulator or fit."""
from pathlib import Path
import os,json,subprocess,sys,datetime
from concurrent.futures import ThreadPoolExecutor
H=Path(__file__).resolve().parent
assert json.loads((H/'minimum_prefix_transition_navigation_status_v1.json').read_text())['status']=='finished'
USD='/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310'
env=dict(os.environ,LD_LIBRARY_PATH=USD+'/bin',PYTHONPATH=USD+':/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm:'+str(H),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_minimum_prefix')
S=H/'minimum_prefix_postclose_verification_status_v1.json';assert not S.exists()
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='running',not_new_physics=True,checks=[]);S.write_text(json.dumps(r,indent=2)+'\n')
def run(name):
 with (H/(name+'.log')).open('w') as log:
  p=subprocess.run([sys.executable,str(H/(name+'.py'))],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=500)
 return dict(name=name,exit_code=p.returncode)
names=['verify_minimum_prefix_transition_runtime_v1','verify_minimum_prefix_transition_memory_v1','verify_minimum_prefix_transition_boundary_v1','assess_minimum_prefix_transition_source_v1','assess_minimum_prefix_transition_membership_v1','verify_minimum_prefix_transition_policy_causal_v2','verify_minimum_prefix_transition_online_shadow_v1']
with ThreadPoolExecutor(max_workers=2) as pool:
 for result in pool.map(run,names):
  r['checks'].append(result);S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(result),flush=True)
if all(c['exit_code']==0 for c in r['checks']):r['checks'].append(run('assess_minimum_prefix_transition_progress_v2'))
r.update(status='finished' if all(c['exit_code']==0 for c in r['checks']) else 'verification_failed',end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True)
sys.exit(0 if r['status']=='finished' else 1)
