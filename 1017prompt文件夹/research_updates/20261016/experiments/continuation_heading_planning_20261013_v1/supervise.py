from pathlib import Path
import json,sys,os,hashlib,datetime
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(H),str(V)];from durable_stage_supervision_v1 import run_durable
from probe_cleanup import seal_existing
from score import score
import audit_actual
audit_actual.D=D
ad=json.loads((D/'admission.json').read_text());assert ad['launch_admitted'] and ad['cap']==2
for file,dig in ad['sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==dig,file
state=dict(status='reserved',cap=2,used=0,recovery_used=0,fit=0,parent_pid=os.getpid(),groups=[]);ledger=D/'ledger.json'
with ledger.open('x') as f:json.dump(state,f);f.flush();os.fsync(f.fileno())
def save():
 p=ledger.with_suffix('.tmp');p.write_text(json.dumps(state,indent=2)+'\n');os.replace(p,ledger)
paths=[D,V,P,H,W/'experiments/oracle_feasible_route_20260929_v9/hybrid',W/'experiments/bilateral_temporal_geometry_navigation_20261003_v1',W/'experiments/clean_visual_hard0_intervention_20261003_v1',W/'experiments/visible_surface_connectivity_20261004_v1']
env={**os.environ,'PYTHONPATH':os.pathsep.join(map(str,paths)),'PYTHONDONTWRITEBYTECODE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MPLCONFIGDIR':'/tmp/mpl_continuation','TORCH_COMPILE_DISABLE':'1'}
for arm in ['baseline_e1','candidate_e1']:
 state['used']+=1;state['status']='running';g=dict(arm=arm,state='reserved',utc=datetime.datetime.now(datetime.timezone.utc).isoformat());state['groups'].append(g);save()
 r=run_durable([sys.executable,str(D/(arm+'_driver.py')),'--condition','e1'],D/(arm+'_status.json'),D/(arm+'_log.log'),timeout=1800,grace=45,env=env);g['execution']=r;save();out=D/(arm+'_run')
 if out.exists():seal_existing(out,{'execution_status':r['status'],'returncode':r['child_returncode']})
 if r['status']!='finished':state['status']='execution_stopped';state['unattempted_closed']=2-state['used'];save();break
 raw=audit_actual.audit(arm);g['raw_audit']=raw;(D/(arm+'_raw_audit.json')).write_text(json.dumps(raw,indent=2)+'\n');g['assessment']=score(out);(D/(arm+'_assessment.json')).write_text(json.dumps(g['assessment'],indent=2)+'\n');g['state']='closed';save()
 if arm=='baseline_e1' and not (g['assessment']['depth_scoring']['pass_gate'] and g['assessment']['unavailable_contacts']==0):state['status']='baseline_source_not_admitted';state['unattempted_closed']=1;save();break
else:state['status']='pair_closed_needs_effect_audit';save()
print(json.dumps(state,indent=2))
