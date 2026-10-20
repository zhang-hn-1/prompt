"""Independent measured XY/contact outcomes; no policy cursor or source accuracy admission."""
import numpy as np

def assess(states,contacts,start,goal,budget_s=35.,goal_threshold_m=.2):
 start=np.asarray(start,float);goal=np.asarray(goal,float);ss=[s for s in states if -1e-5<=float(s['time_s'])<=budget_s+1e-5]
 if not ss:raise ValueError('no scored states')
 times=np.asarray([s['time_s'] for s in ss]);xy=np.asarray([s['position'][:2] for s in ss],float)
 if np.any(np.diff(times)<=0) or not np.isfinite(xy).all():raise ValueError('invalid measured sequence')
 cc=[c for c in contacts if -1e-5<=c['time_s']<=budget_s+1e-5];hits=[c['time_s'] for c in cc if c['available'] and c['collision']];hit=min(hits) if hits else None
 remaining=np.linalg.norm(xy-goal,axis=1);arrivals=np.flatnonzero(remaining<=goal_threshold_m);arrival=float(times[arrivals[0]]) if len(arrivals) else None
 end=min(float(times[-1]),hit if hit is not None else budget_s,arrival if arrival is not None else budget_s);use=times<=end+1e-5;j=int(np.flatnonzero(use)[-1]);usedcc=[c for c in cc if c['time_s']<=end+1e-5];available=bool(usedcc) and all(c['available'] for c in usedcc)
 if hit is not None and (arrival is None or hit<=arrival+1e-5):outcome='collision'
 elif not available:outcome='contact_evidence_unavailable'
 elif arrival is not None:outcome='success'
 elif times[-1]<budget_s-1e-5:outcome='early_stop'
 else:outcome='timeout'
 unit=(goal-start)/np.linalg.norm(goal-start);p=xy[:j+1];return dict(outcome=outcome,success=outcome=='success',budget_s=budget_s,scored_end_s=end,metric_state_time_s=float(times[j]),goal_threshold_m=goal_threshold_m,goal_remaining_m=float(remaining[j]),goal_distance_reduction_m=float(np.linalg.norm(start-goal)-remaining[j]),along_start_goal_axis_m=float((xy[j]-start)@unit),path_length_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),contact_time_s=hit,arrival_time_s=arrival,contacts_all_available=available,contact_checks=len(usedcc),measured_states=len(p),not_based_on_path_index=True,not_strict_visual_admission=True)

def observation_gap_diagnostics(source_rows,policy_rows):
 n=len(source_rows);empty=[a for a in source_rows if a['points']==0];span=0.;first=None
 for a in source_rows:
  if a['points']==0:
   if first is None:first=a['time_s']
   span=max(span,float(a['time_s']-first))
  else:first=None
 unknown=[a['footprint_unknown'] for a in policy_rows if 'footprint_unknown' in a]
 return dict(source_epochs=n,empty_source_epochs=len(empty),empty_epoch_fraction=len(empty)/n if n else None,maximum_sampled_empty_span_s=span,empty_sample_times_s=[a['time_s'] for a in empty],footprint_unknown_cells_median=float(np.median(unknown)) if unknown else None,footprint_unknown_cells_max=max(unknown) if unknown else None,absence_is_observed_free=False,full_3d_blindzone_measured=False,interpretation='Sampled point-source gaps and map unknown footprint only; cannot distinguish occlusion from matching/selection failure or certify unseen space. Task ground/safe-start priors remain separate; no source accuracy admission.')
