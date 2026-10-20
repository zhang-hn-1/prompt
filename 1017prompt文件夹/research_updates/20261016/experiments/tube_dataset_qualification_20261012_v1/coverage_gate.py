"""Declared dataset policy, not statistical assurance or authenticated record ingestion."""
SPLITS=('development','calibration','evaluation')
PROGRAMS={'stop','forward','left','right'}
STATES={'low','moving'}
def assess(episodes):
 reasons=set();seen=set();counts={s:0 for s in SPLITS}
 for row in episodes:
  if not isinstance(row['group'],str) or not row['group']:reasons.add('invalid_group')
  if row['group'] in seen:reasons.add('group_reused')
  seen.add(row['group'])
  split=row['split']
  if split not in counts:reasons.add('invalid_split')
  else:counts[split]+=1
  planned=row['planned'];attempted=row['attempted'];scorable=row['scorable']
  if type(planned) is not int or type(attempted) is not int or type(scorable) is not int or planned<=0 or not 0<=scorable<=attempted<=planned:reasons.add('invalid_denominator')
  elif scorable/planned<.95:reasons.add('insufficient_complete_fraction')
  if row.get('support_from')!='complete_scorable_only':reasons.add('invalid_support_origin')
  if not {(a,b) for a in PROGRAMS for b in STATES}.issubset({tuple(cell) for cell in row.get('support_cells',[])}):reasons.add('missing_joint_support')
  if not PROGRAMS.issubset(row['programs']) or not STATES.issubset(row['state_bins']):reasons.add('missing_group_support')
 if any(counts[s]<10 for s in SPLITS):reasons.add('insufficient_episode_groups')
 return dict(data_gate_pass=not reasons,reasons=sorted(reasons),groups_per_split=counts,fit_admitted=False,consumer_admitted=False,statistical_coverage_guaranteed=False)
