"""Execution closure precedes scientific classification; no incomplete run can pass35."""
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence

def classify(status, sealed, identity, states, contacts, physics_clock, n):
 result=dict(classification='unassessed_execution',complete_35=False,scored_end_s=None)
 if status.get('status')!='finished' or status.get('child_returncode')!=0:return result
 result['classification']='unassessed_evidence'
 if not sealed:return result
 try:
  if identity.get('actions')!=n or identity.get('captures')!=n+1:raise ValueError('identity counts')
  normalized=complete_execution_evidence(states,contacts,physics_clock,n)
 except (ValueError,KeyError,TypeError):return result
 hits=[c['time_s'] for c in normalized if c['collision']]
 if hits:
  result.update(classification='measured_collision_reject',scored_end_s=min(hits))
 elif n==350 and identity.get('complete') is True:
  result.update(classification='complete_35',complete_35=True,scored_end_s=states[-1]['time_s'])
 return result


def shared_execution_prefix(a,b,fork,b0_n,paced_n):
 import json
 if not 0<=fork<min(b0_n,paced_n):return False
 return all(json.loads((a/'execution'/f'frame_{i:03d}.json').read_text())==json.loads((b/'execution'/f'frame_{i:03d}.json').read_text()) for i in range(fork+1))
