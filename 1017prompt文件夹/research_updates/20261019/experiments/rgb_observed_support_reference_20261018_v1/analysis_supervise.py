from pathlib import Path
import json,hashlib,os,sys
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg=json.loads((D/'analysis_protocol.json').read_text())
for p,h in cfg['sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
f=D/'ledger.json';l=json.loads(f.read_text());assert l['dispatch_used']==2 and l['dispatch_cap']==4 and not any(x['stage']=='failure_analysis' for x in l['runs'])
l['dispatch_used']+=1;l['runs'].append(dict(stage='failure_analysis',parent_pid=os.getpid(),status='starting'));f.write_text(json.dumps(l,indent=2)+'\n')
s=run_group([sys.executable,'-B',str(D/'failure_analysis.py')],D/'analysis_status.json',D/'analysis.log',timeout=30,grace=5,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
l['runs'][-1].update(status=s['status'],execution=s);f.write_text(json.dumps(l,indent=2)+'\n');assert s['status']=='finished' and s['child_returncode']==0
