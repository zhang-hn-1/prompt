from pathlib import Path
import sys,os,json,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg=json.loads((D/'repair_protocol.json').read_text())
for p,h in cfg['sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
f=D/'ledger.json';l=json.loads(f.read_text());assert l['dispatch_used']==3 and l['dispatch_cap']==4
l['dispatch_used']+=1;l['runs'].append(dict(stage='controlled_validity_repair_check',parent_pid=os.getpid(),status='starting'));f.write_text(json.dumps(l,indent=2)+'\n')
s=run_group([sys.executable,'-B',str(D/'repair_check.py')],D/'repair_status.json',D/'repair.log',timeout=30,grace=5,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
l['runs'][-1].update(status=s['status'],execution=s);f.write_text(json.dumps(l,indent=2)+'\n');assert s['status']=='finished' and s['child_returncode']==0
