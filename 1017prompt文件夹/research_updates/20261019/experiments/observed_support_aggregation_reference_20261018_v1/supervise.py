from pathlib import Path
import os,sys,json,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
stage=sys.argv[1];assert stage in ['red','qualification']
cfg=json.loads((D/('protocol_preimplementation.json' if stage=='red' else 'protocol.json')).read_text())
for p,h in cfg['code_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
lp=D/'ledger.json';l=json.loads(lp.read_text()) if lp.exists() else dict(dispatch_cap=2,dispatch_used=0,recovery_cap=1,recovery_used=0,runs=[],fit=0,RGB_source_calls=0,physics=0,navigation=0)
assert l['dispatch_used']<2 and stage not in [a['stage'] for a in l['runs']]
l['dispatch_used']+=1;l['runs'].append(dict(stage=stage,status='starting',parent_pid=os.getpid(),kernel_sha256=hashlib.sha256((D/'kernel.py').read_bytes()).hexdigest()));lp.write_text(json.dumps(l,indent=2)+'\n')
status=run_group([sys.executable,'-B',str(D/('red_check.py' if stage=='red' else 'qualify.py'))],D/(stage+'_status.json'),D/(stage+'.log'),timeout=30,grace=5,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
l['runs'][-1].update(status=status['status'],execution=status,expected_red=stage=='red' and status['child_returncode']==1 and 'NotImplementedError' in (D/'red.log').read_text());lp.write_text(json.dumps(l,indent=2)+'\n')
assert l['runs'][-1]['expected_red'] if stage=='red' else status['status']=='finished' and status['child_returncode']==0
