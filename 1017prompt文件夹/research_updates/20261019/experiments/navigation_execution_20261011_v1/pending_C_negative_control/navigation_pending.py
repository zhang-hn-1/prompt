"""Prepared C negative-control wiring. No launch without direct human exception."""
from pathlib import Path
import json,hashlib,sys,importlib.util,os,time
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 proposal=json.loads((D/'proposal.json').read_text())
 approval_path=D/'approval.json'
 if not approval_path.exists():raise RuntimeError('No direct user budget exception: C remains closed')
 approval=json.loads(approval_path.read_text())
 if approval.get('user_explicit_authorization') is not True or approval.get('proposal_sha256')!=sha(D/'proposal.json'):
  raise RuntimeError('Explicit human exception must bind this exact proposal')
 # A human exception alone does not bypass interface, budget or durable launch admission.
 ap=D/'execution_admission.json'
 if not ap.exists():raise RuntimeError('C execution admission missing; do not import runtime')
 admitted=json.loads(ap.read_text())
 if admitted.get('proposal_sha256')!=sha(D/'proposal.json') or admitted.get('navigation_protocol_sha256')!=sha(D/'navigation_protocol.json'):
  raise RuntimeError('Execution admission identity mismatch')
 interface_path=D/'C_interface_result.json'
 ledger_path=D/'navigation_ledger.json'
 if not interface_path.exists() or not ledger_path.exists():raise RuntimeError('C interface/ledger absent')
 interface=json.loads(interface_path.read_text());ledger=json.loads(ledger_path.read_text())
 if not interface.get('pass_required_C_interface') or interface.get('proposal_sha256')!=sha(D/'proposal.json') or admitted.get('interface_sha256')!=sha(interface_path):
  raise RuntimeError('C-specific interface prerequisite not passed or not identity-bound')
 if ledger.get('cap')!=1 or ledger.get('used')!=1 or ledger.get('recovery_cap')!=0 or ledger.get('status')!='reserved' or ledger.get('proposal_sha256')!=sha(D/'proposal.json') or ledger.get('parent_pid')!=os.getppid() or admitted.get('ledger_sha256')!=sha(ledger_path):
  raise RuntimeError('One authorized C budget and current durable parent required')
 # Supervisor may write running status immediately after Popen; allow only a bounded metadata wait.
 deadline=time.monotonic()+2.;registered=False
 while time.monotonic()<deadline:
  sp=D/'navigation_status.json'
  try:
   status=json.loads(sp.read_text())
   registered=status.get('status')=='running' and status.get('child_pid')==os.getpid() and status.get('parent_pid')==os.getppid()
  except (FileNotFoundError,json.JSONDecodeError):registered=False
  if registered:break
  time.sleep(.01)
 if not registered:raise RuntimeError('Durable child and parent PID registration missing')
 # Only after all admission guards may runtime imports or sensor/model access begin.
 for p,h in proposal['frozen_identity'].items():
  if sha(p)!=h:raise RuntimeError('frozen identity changed: '+p)
 root=Path(proposal['workspace']);base=Path(proposal['baseline_root']);cm=Path(proposal['C_root'])
 sys.path[:0]=[str(base),str(cm),str(D)]
 from metadata_adapter import add_view_metadata
 spec=importlib.util.spec_from_file_location('pending_C_original_navigation',str(base/'navigation.py'))
 nav=importlib.util.module_from_spec(spec);spec.loader.exec_module(nav)
 memoryspec=importlib.util.spec_from_file_location('pending_C_exact_memory',str(cm/'memory.py'))
 memory=importlib.util.module_from_spec(memoryspec);memoryspec.loader.exec_module(memory)
 from source import LOCAL
 original_points=nav.points
 def points(pred,body,frame,t):
  records,arrays=original_points(pred,body,frame,t)
  return add_view_metadata(records,body,LOCAL),arrays
 nav.D=D;nav.points=points;nav.RepeatedExposureMemoryGrid=memory.ViewSupportMemoryGrid
 nav.main()
if __name__=='__main__':main()
