from pathlib import Path
import json,numpy as np
from roll_pid import BankPID
h=Path(__file__).resolve().parent;cfg=json.loads((h/'selected_candidate.json').read_text());out=[]
for p in sorted(h.glob('navigation_*/feedback/bank_feedback_*.json')):
 rows=json.loads(p.read_text());pid=BankPID(**cfg['gains']);blocked=False;errors=[]
 for r in rows:
  pid.step([r['main_request_rad_s'],r['aux_feedforward_rad']],r['bank_rad'],downstream_blocked=blocked);errors.append(abs(pid.last['pid_protocol_aux_rad']-r['pid_protocol_aux_rad']));blocked=r['supervisor_active']
 assert max(errors)<1e-12
 mapping=json.loads((p.parent/p.name.replace('bank_feedback_','mapping_')).read_text());assert len(mapping)*5==len(rows)
 out.append(dict(episode=p.stem,ticks=len(rows),selected_gains_exact_output_replay=True,max_aux_error=max(errors),online_mapping_blocks=len(mapping)))
(h/'execution_audit.json').write_text(json.dumps(out,indent=2));print('Selected PID gains verified against all recorded navigation ticks:',sum(r['ticks'] for r in out))
