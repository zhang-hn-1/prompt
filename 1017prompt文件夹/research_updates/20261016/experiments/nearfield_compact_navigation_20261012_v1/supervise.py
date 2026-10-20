from pathlib import Path
import json,sys,os,hashlib,datetime
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(H));from durable_stage_supervision_v1 import run_durable
from probe_cleanup import seal_existing
from score import score
ad=json.loads((D/'admission.json').read_text());assert ad['launch_admitted'] and ad['cap']==3
for p,h in ad['source_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
ledger=D/'ledger.json'
state=dict(status='reserved',used=0,cap=3,retry=0,parent_pid=os.getpid(),groups=[])
with ledger.open('x') as f:json.dump(state,f);f.flush();os.fsync(f.fileno())
def save():
 t=ledger.with_suffix('.tmp');t.write_text(json.dumps(state,indent=2)+'\n');os.replace(t,ledger)
paths=[D,P,H,W/'experiments/oracle_feasible_route_20260929_v9/hybrid',W/'experiments/bilateral_temporal_geometry_navigation_20261003_v1',W/'experiments/clean_visual_hard0_intervention_20261003_v1',W/'experiments/visible_surface_connectivity_20261004_v1']
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(map(str,paths)),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_nearfield_rig',TORCH_COMPILE_DISABLE='1')
try:
 for key in ['candidate_e2','candidate_e0','candidate_e1']:
  state['used']+=1;state['status']='running';g=dict(arm=key,state='attempt_reserved',utc=datetime.datetime.now(datetime.timezone.utc).isoformat());state['groups'].append(g);save()
  r=run_durable([sys.executable,str(D/(key+'_driver.py')),'--condition',key[-2:]],D/(key+'_status.json'),D/(key+'_log.log'),timeout=1800,grace=45,env=env);g['execution']=r;save()
  out=D/(key+'_run')
  if out.exists():seal_existing(out,{'execution_status':r['status'],'returncode':r['child_returncode']})
  if r['status']!='finished':state['status']='execution_stopped';save();break
  result=score(out);(D/(key+'_assessment.json')).write_text(json.dumps(result,indent=2)+'\n');g['assessment']=result;save()
  if key=='candidate_e2' and not result['retain']:
   state['status']='candidate_rejected';state['remaining_closed']=3-state['used'];save();break
 else:state['status']='finished';save()
except BaseException as e:
 state['status']='supervisor_interrupted' if isinstance(e,(SystemExit,KeyboardInterrupt)) else 'supervisor_failed';state['exception']=type(e).__name__;save();raise
