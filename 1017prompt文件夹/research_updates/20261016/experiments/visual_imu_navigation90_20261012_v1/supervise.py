from pathlib import Path
import json,sys,os,hashlib,datetime
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(H));from durable_stage_supervision_v1 import run_durable
from probe_cleanup import seal_existing
ad=json.loads((D/'admission.json').read_text())
assert ad['launch_admitted'] and ad['cap']==3 and ad['retry']==0
for p,h in ad['source_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
ledger=D/'ledger.json'
with ledger.open('x') as f:json.dump({'status':'reserved','used':0,'cap':3,'retry':0,'parent_pid':os.getpid(),'groups':[]},f);f.flush();os.fsync(f.fileno())
state=json.loads(ledger.read_text())
def save():
 t=ledger.with_suffix('.tmp');t.write_text(json.dumps(state,indent=2)+'\n');os.replace(t,ledger)
paths=[P,H,W/'experiments/oracle_feasible_route_20260929_v9/hybrid',W/'experiments/bilateral_temporal_geometry_navigation_20261003_v1',W/'experiments/clean_visual_hard0_intervention_20261003_v1',W/'experiments/visible_surface_connectivity_20261004_v1']
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(map(str,paths)),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_nav90',TORCH_COMPILE_DISABLE='1')
try:
 for scene in ['e0','e1','e2']:
  state['used']+=1;state['status']='running';state['groups'].append({'scenario':scene,'state':'attempt_reserved','utc':datetime.datetime.now(datetime.timezone.utc).isoformat()});save()
  r=run_durable([sys.executable,str(D/(scene+'_driver.py')),'--condition',scene],D/(scene+'_status.json'),D/(scene+'_log.log'),timeout=1800,grace=45,env=env)
  state['groups'][-1]['execution']=r
  out=D/(scene+'_run')
  if out.exists():seal_existing(out,{'execution_status':r['status'],'returncode':r['child_returncode']})
  if r['status']!='finished':state['status']='execution_stopped';save();break
  state['groups'][-1]['termination']=json.loads((out/'termination.json').read_text());save()
 else:state['status']='finished';save()
except BaseException as e:
 state['status']='supervisor_interrupted' if isinstance(e,(SystemExit,KeyboardInterrupt)) else 'supervisor_failed';state['exception']=type(e).__name__;save();raise
