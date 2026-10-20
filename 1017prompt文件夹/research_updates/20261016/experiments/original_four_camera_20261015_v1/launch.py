from pathlib import Path
import sys,os,json,hashlib,datetime
D=Path(__file__).resolve().parent;sys.path[:0]=[str(D),str(D.parent/'stereo_memory_navigation_20261004_v1')]
from durable_stage_supervision_v1 import run_durable
from four_camera import validate_complete
cfg=json.loads((D/'protocol.json').read_text());ledger=json.loads((D/'ledger.json').read_text());assert ledger['physical_used']<ledger['physical_cap'];attempt=ledger['physical_used'];assert attempt in [0,1]
if attempt==1:assert ledger['recovery_used']<ledger['recovery_cap'];ledger['recovery_used']+=1
name=['native_run','recovery_run'][attempt];status=D/(name+'_status.json');assert not status.exists() and not (D/name).exists()
ledger['physical_used']+=1;ledger['status']='running';ledger['dispatches'].append(dict(attempt=attempt+1,name=name,registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),timeout_s=120,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'protocol.json',D/'native_probe.py',D/'four_camera.py',D/'launch.py',D.parent/'camera_exposure_contract_20261015_v1/exposure_contract.py']}));(D/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310');env={**os.environ,'PYTHONPATH':str(usd)+':/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm','LD_LIBRARY_PATH':str(usd/'bin')+':'+os.environ.get('LD_LIBRARY_PATH',''),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','PYTHONUTF8':'1','MPLCONFIGDIR':'/tmp/ball_nav_mpl','TORCH_COMPILE_DISABLE':'1'}
r=run_durable([sys.executable,str(D/'native_probe.py'),name],status,D/(name+'.log'),timeout=120,grace=20,env=env)
try:
 seal=validate_complete(r,D/name);logical=dict(complete=True,frames=len(seal['rows']),camera_count=4,model=0,navigation=0)
except Exception as e:logical=dict(complete=False,reason=str(e),model=0,navigation=0)
(D/(name+'_completion.json')).write_text(json.dumps(logical,indent=2)+'\n');ledger['status']='complete' if logical['complete'] else 'failed';ledger['dispatches'][-1]['terminal']=r;ledger['dispatches'][-1]['logical']=logical
if logical['complete'] or ledger['physical_used']==ledger['physical_cap']:ledger['budget_closed']=True
(D/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n');print(json.dumps(logical),flush=True)
if not logical['complete']:sys.exit(1)
