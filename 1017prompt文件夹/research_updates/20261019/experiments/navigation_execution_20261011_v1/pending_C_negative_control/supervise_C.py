"""Only the three explicitly approved stages; no retries or old ledger changes."""
from pathlib import Path
import os,sys,json,hashlib,datetime
D=Path(__file__).resolve().parent;W=D.parents[2]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def main():
 stage=sys.argv[1];assert stage in ['interface','navigation','score'];cfg=json.loads((D/'execution_protocol.json').read_text());approval=json.loads((D/'approval.json').read_text());assert approval['user_explicit_authorization'] and approval['proposal_sha256']==sha(D/'proposal.json')
 for p,h in cfg['source_sha256'].items():assert sha(p)==h,p
 for p,h in json.loads((D/'proposal.json').read_text())['frozen_identity'].items():assert sha(p)==h,p
 lp=D/'execution_ledger.json';ledger=json.loads(lp.read_text());assert ledger['used']<3 and stage not in [r['stage'] for r in ledger['runs']]
 if stage=='navigation':
  s=json.loads((D/'interface_status.json').read_text());i=json.loads((D/'C_interface_result.json').read_text());assert s['status']=='finished' and s['child_returncode']==0 and i['pass_required_C_interface'];assert not (D/'e1_run').exists() and not (D/'navigation_ledger.json').exists()
 if stage=='score':
  s=json.loads((D/'navigation_status.json').read_text());assert s['status']=='finished' and s['child_returncode']==0 and not s['remaining_live_group_pids'];assert json.loads((D/'e1_run/capture_seal.json').read_text())['complete'] and json.loads((D/'e1_run/source_seal.json').read_text())['complete']
 ledger['used']+=1;ledger['runs'].append(dict(stage=stage,status='reserved',parent_pid=os.getpid(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat()));put(lp,ledger)
 if stage=='navigation':
  nl=dict(cap=1,used=1,recovery_cap=0,recovery_used=0,status='reserved',parent_pid=os.getpid(),proposal_sha256=sha(D/'proposal.json'),source_physical_cumulative=3,arm='C_original_negative_control',direct_user_exception_sha256=sha(D/'approval.json'));put(D/'navigation_ledger.json',nl)
  put(D/'execution_admission.json',dict(proposal_sha256=sha(D/'proposal.json'),navigation_protocol_sha256=sha(D/'navigation_protocol.json'),interface_sha256=sha(D/'C_interface_result.json'),ledger_sha256=sha(D/'navigation_ledger.json'),execution_protocol_sha256=sha(D/'execution_protocol.json'),approval_sha256=sha(D/'approval.json')))
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=cfg['sdk'],LD_LIBRARY_PATH=cfg['sdk']+'/bin'+(':'+os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else ''),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',DISPLAY=':0',XDG_RUNTIME_DIR='/run/user/1000',MPLCONFIGDIR='/tmp/mpl_C_negativecontrol',TORCH_COMPILE_DISABLE='1')
 sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'));from group_supervision import run_group
 script={'interface':'C_interface.py','navigation':'navigation_pending.py','score':'score_C.py'}[stage]
 s=run_group([sys.executable,'-B',str(D/script)],D/(stage+'_status.json'),D/(stage+'.log'),timeout=cfg['timeouts_s'][stage],grace=45 if stage=='navigation' else 15,env=env)
 ledger['runs'][-1].update(status=s['status'],execution=s);put(lp,ledger)
 if stage=='navigation':nl.update(status=s['status'],execution=s);put(D/'navigation_ledger.json',nl)
 assert s['status']=='finished' and s['child_returncode']==0,s
if __name__=='__main__':main()
