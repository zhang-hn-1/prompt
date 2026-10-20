from pathlib import Path
import sys,json,os,hashlib,datetime
D=Path(__file__).resolve().parent;W=D.parents[1];O=W/'experiments/fixed_source_parallel_compute_20261017_v1';sys.path.insert(0,str(O))
from group_supervision import run_group
load=lambda p:json.loads(Path(p).read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();put=lambda p,x:p.write_text(json.dumps(x,indent=2)+'\n')
cfg=load(D/'protocol.json')
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
pr=load(cfg['prerequisite_result']);assert pr['complete'] and pr['decision']=='FULL470_OUTPUT_EQUIVALENCE_PASSED_LOCAL_LATENCY_CHECK' and all(pr['gates'].values());assert load(O/'status.json')['status']=='finished'
assert not (D/'ledger.json').exists();put(D/'prerequisite_receipt.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),path=cfg['prerequisite_result'],sha256=sha(cfg['prerequisite_result']),output_arrays_byte_exact=pr['output_arrays_byte_exact']))
s=dict(dispatch_cap=1,dispatch_used=1,recovery_cap=1,recovery_used=0,full_source_invocations_cap=8,full_mask_invocations_cap=470,constructor_static_mask_projections=4,fit=0,learned_inference=0,physics=0,navigation=0,parent_pid=os.getpid(),old_budgets_reopened=False);put(D/'ledger.json',s)
SDK=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
r=run_group([sys.executable,'-B',str(D/'qualify.py')],D/'status.json',D/'run.log',timeout=300,grace=15,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH=str(SDK),LD_LIBRARY_PATH=str(SDK/'bin')+':'+os.environ.get('LD_LIBRARY_PATH','')));s.update(status=r['status'],execution=r);put(D/'ledger.json',s);assert r['status']=='finished' and r['child_returncode']==0,r
