from pathlib import Path
import sys,os,json,hashlib,datetime
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(H),str(V)]
from durable_stage_supervision_v1 import run_durable
from probe_cleanup import seal_existing
from score import score
ad=json.loads((D/'collection_admission.json').read_text())
for p,h in ad['launch_files_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
for ep in ['e0','e1']:
 c=json.loads((D/f'data_{ep}_protocol.json').read_text())
 for key in ['source_sha256','input_sha256']:
  for p,h in c[key].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
state=dict(status='reserved',purpose='new_current_error_training_qualification_not_old_navigation_candidate_rerun',cap=4,used=0,fit=0,recovery_used=0,parent_pid=os.getpid(),groups=[])
ledger=D/'collection_ledger.json'
with ledger.open('x') as f:json.dump(state,f);f.flush();os.fsync(f.fileno())
def save():
 t=ledger.with_suffix('.tmp');t.write_text(json.dumps(state,indent=2)+'\n');os.replace(t,ledger)
paths=[D,V,P,H,W/'experiments/risk_height_navigation_20261012_v1',W/'experiments/oracle_feasible_route_20260929_v9/hybrid',W/'experiments/bilateral_temporal_geometry_navigation_20261003_v1',W/'experiments/clean_visual_hard0_intervention_20261003_v1',W/'experiments/visible_surface_connectivity_20261004_v1']
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(map(str,paths)),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_source_confidence',TORCH_COMPILE_DISABLE='1')
for ep in ['e0','e1']:
 state['used']+=1;state['status']='running';g=dict(arm='data_'+ep,state='attempt_reserved',utc=datetime.datetime.now(datetime.timezone.utc).isoformat());state['groups'].append(g);save()
 r=run_durable([sys.executable,str(D/f'data_{ep}_driver.py'),'--condition',ep],D/f'data_{ep}_status.json',D/f'data_{ep}_log.log',timeout=1800,grace=45,env=env);g['execution']=r;save();out=D/f'data_{ep}_run'
 if out.exists():seal_existing(out,{'execution_status':r['status'],'returncode':r['child_returncode'],'new_data_objective':True})
 if r['status']!='finished':state['status']='execution_stopped';save();break
 g['navigation_descriptive']=score(out);(D/f'data_{ep}_assessment.json').write_text(json.dumps(g['navigation_descriptive'],indent=2)+'\n');g['state']='data_terminal_needs_coverage_and_raw_audit';save()
else:state['status']='collected_requires_qualification';save()
print(json.dumps(state,indent=2))
