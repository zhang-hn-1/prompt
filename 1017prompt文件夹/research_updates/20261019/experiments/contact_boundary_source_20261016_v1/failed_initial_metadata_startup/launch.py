from pathlib import Path
import os,sys,json,datetime
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable
from validate import validate
cfg=json.loads((D/'protocol.json').read_text());ledger=json.loads((D/'ledger.json').read_text())
condition=sys.argv[1];assert condition in cfg['acquisitions']
assert ledger['physical_used']<ledger['physical_cap'] and ledger['acquisition_used']<ledger['acquisition_cap']
assert not ledger.get('budget_closed',False) and not (D/(condition+'_status.json')).exists()
assert not (D/(condition+'_run')).exists()
if condition=='e1':
    assert json.loads((D/'e0_qualification.json').read_text())['all_required_gates_pass']
ledger['physical_used']+=1;ledger['acquisition_used']+=1;ledger['status']='running'
ledger['dispatches'].append(dict(condition=condition,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    purpose='fresh four-camera acquisition only',physical_attempt=ledger['physical_used']))
def save():
    p=D/'ledger.tmp';p.write_text(json.dumps(ledger,indent=2)+'\n');os.replace(p,D/'ledger.json')
save()
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
env={**os.environ,'PYTHONPATH':str(usd)+':'+str(W/'experiments/stereo_memory_navigation_20261004_v1')+':/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',
    'LD_LIBRARY_PATH':str(usd/'bin')+':'+os.environ.get('LD_LIBRARY_PATH',''),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1',
    'PYTHONDONTWRITEBYTECODE':'1','PYTHONUTF8':'1','MPLCONFIGDIR':'/tmp/contact_boundary_mpl','TORCH_COMPILE_DISABLE':'1',
    'DISPLAY':os.environ.get('DISPLAY',':0'),'XDG_RUNTIME_DIR':os.environ.get('XDG_RUNTIME_DIR','/run/user/1000')}
r=run_durable([sys.executable,str(D/'collect.py'),condition],D/(condition+'_status.json'),D/(condition+'.log'),
    timeout=cfg['wall_timeout_per_physical_s'],grace=30,env=env)
ledger['dispatches'][-1]['terminal']=r
try:
    completion=validate(condition,r);ledger['status']='acquisition_complete_pending_qualification'
except Exception as e:
    completion=dict(complete=False,reason=str(e),model=0,fit=0,navigation=0);ledger['status']='acquisition_failed'
(D/(condition+'_completion.json')).write_text(json.dumps(completion,indent=2)+'\n')
ledger['dispatches'][-1]['logical']=completion;save();print(completion,flush=True)
if not completion['complete']:sys.exit(1)
