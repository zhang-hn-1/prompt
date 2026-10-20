from pathlib import Path
import sys,json,os,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
stage=sys.argv[1];assert stage in ['red','fixtures','predict','score']
cfg=json.loads((D/('protocol_preimplementation.json' if stage=='red' else 'protocol.json')).read_text())
for sec in ['code_sha256','input_sha256']:
 for p,h in cfg[sec].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
f=D/'ledger.json';l=json.loads(f.read_text()) if f.exists() else dict(dispatch_cap=4,dispatch_used=0,recovery_cap=1,recovery_used=0,runs=[],fit=0,physics=0,navigation=0)
assert l['dispatch_used']<4 and stage not in [x['stage'] for x in l['runs']]
l['dispatch_used']+=1;l['runs'].append(dict(stage=stage,parent_pid=os.getpid(),status='starting'));f.write_text(json.dumps(l,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH=cfg['sdk'],LD_LIBRARY_PATH=cfg['sdk']+'/bin'+(':'+os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else ''))
s=run_group([sys.executable,'-B',str(D/(stage+'_check.py' if stage=='red' else stage+'.py'))],D/(stage+'_status.json'),D/(stage+'.log'),timeout=900 if stage=='predict' else 90,grace=10,env=env)
expected=stage=='red' and s['child_returncode']==1 and 'NotImplementedError' in (D/'red.log').read_text();l['runs'][-1].update(status=s['status'],execution=s,expected_red=expected);f.write_text(json.dumps(l,indent=2)+'\n')
assert expected if stage=='red' else s['status']=='finished' and s['child_returncode']==0
