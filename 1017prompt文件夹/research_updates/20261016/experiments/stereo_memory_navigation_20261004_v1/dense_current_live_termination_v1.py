"""Frozen original35 stop or original acquisition20 reject; no rescue/tail."""
import math
def validate_termination(term,metric,identity,requests):
 label=term['termination'];end=float(term['time_s'])
 if not math.isfinite(end) or abs(end-requests*.1)>2e-6:raise ValueError('actual termination clock')
 if label=='REJECT_INITIAL_VIEW_HANDOFF_TIMEOUT':
  info=identity.get('front_view_acquisition',{})
  if not info.get('failed') or info.get('handed_off') or abs(end-20.)>2e-6 or metric['outcome']!='early_stop':raise ValueError('original handoff abort evidence required')
  return 'initial_view_handoff_reject'
 expected={'success':'success','collision':'collision','timeout':'timeout35'}.get(metric['outcome'])
 if expected!=label or (requests<350 and metric['outcome'] not in ('success','collision')):raise ValueError('incomplete or mismatched execution outcome')
 return 'original35_scored_completion'
