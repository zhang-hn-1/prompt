"""Predeclared complete-development navigation gates, not source accuracy gates."""
def gates(b0,candidate,*,changed,contract):
 return dict(runtime_source_contract=bool(contract),actual_action_effect=bool(changed),
  no_contact=bool(candidate['raw_contacts_zero']),arrival=bool(candidate['success']),
  net_progress_improves=candidate['along_start_goal_axis_m']>b0['along_start_goal_axis_m']+1e-6,
  cross_rms=candidate['cross_rms_m']<=max(.3,1.25*b0['cross_rms_m']),
  cross_peak=candidate['cross_peak_m']<=max(.3,1.25*b0['cross_peak_m']),
  latency=candidate['wall_median_s']<=1.25*b0['wall_median_s'] and candidate['wall_p90_s']<=1.25*b0['wall_p90_s'])
