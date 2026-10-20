"""Bounded current-state diagnostic on two closed sequences. No new actor or score gate."""
from pathlib import Path
import json,hashlib
import numpy as np
from local_terminal_capture_context_v1 import LocalTerminalCapture
H=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def assess(arm):
 d=H/f'fixed_reference_pacing_knownpose_{arm}_run_v3'
 states=json.loads((d/'states.json').read_text());rows=json.loads((d/'policy_records.json').read_text())
 c=LocalTerminalCapture();samples=[];history_hashes={}
 for i,row in enumerate(rows):
  if i==0 or not rows[i-1].get('installed_reference_xy_after_act'):continue
  previous=rows[i-1];p=d/'history'/f'frame_{i:03d}.npz'
  with np.load(p) as f:velocity=f['proprio_history'][-1,:2].copy()
  history_hashes[str(p)]=sha(p)
  context=c.observe(states[i]['time_s'],states[i]['position'][:2],velocity,previous['installed_reference_xy_after_act'][-1],previous['local_reference_version'])
  # Subsequent issued command is an outcome for diagnosis, never an input to qualification.
  context.update(frame=i,issued_main=float(row['request'][0]),braking_after_act=row['tracker_state_after_act']['braking_state'])
  samples.append(context)
 qualifying=[x for x in samples if x['qualified_now']]
 resumed=[x for x in qualifying if abs(x['issued_main'])>0]
 return dict(samples=samples,summary=dict(qualified_samples=len(qualifying),first_qualified=qualifying[0] if qualifying else None,nonzero_main_while_currently_qualified=len(resumed),first_nonzero_main_while_qualified=resumed[0] if resumed else None,last_before_request_context=samples[-1]),input_sha256={str(d/n):sha(d/n) for n in ['states.json','policy_records.json']},history_sha256=history_hashes)

if __name__=='__main__':
 out=dict(scope='read-only local endpoint/current actual velocity diagnosis; not navigation admission or alternate-plan truth',criteria=dict(endpoint_radius_m=.2,actual_XY_speed_strictly_below_m_s=.05,consecutive_samples=5,sample_dt_s=.1,reset_on_rebound_gap_or_reference_change=True),physical_runs_added=0,candidate_remains_rejected=True,arms={a:assess(a) for a in ['b0','paced']},source_sha256={str(H/n):sha(H/n) for n in ['local_terminal_capture_context_v1.py','assess_local_terminal_capture_v2.py','test_local_terminal_capture_context_v1.py']})
 (H/'local_terminal_capture_assessment_v2.json').write_text(json.dumps(out,indent=2)+'\n')
 print(json.dumps({a:v['summary'] for a,v in out['arms'].items()},indent=2))
