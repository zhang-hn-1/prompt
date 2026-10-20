from pathlib import Path
import sys,os,json,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
stage=sys.argv[1];assert stage in ['interfaces','predict','score'];cfg=json.loads((D/'protocol.json').read_text())
for sec in ['code_sha256','input_sha256']:
 for p,h in cfg[sec].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
f=D/'ledger.json';l=json.loads(f.read_text());assert l['dispatch_used']<l['dispatch_cap'] and stage not in [x['stage'] for x in l['runs']]
l['dispatch_used']+=1;l['runs'].append(dict(stage=stage,status='starting',parent_pid=os.getpid()));f.write_text(json.dumps(l,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH=cfg['sdk'],LD_LIBRARY_PATH=cfg['sdk']+'/bin'+(':'+os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else ''))
s=run_group([sys.executable,'-B',str(D/(stage+'.py'))],D/(stage+'_status.json'),D/(stage+'.log'),timeout=900 if stage=='predict' else 90,grace=10,env=env)
l['runs'][-1].update(status=s['status'],execution=s);f.write_text(json.dumps(l,indent=2)+'\n');assert s['status']=='finished' and s['child_returncode']==0
