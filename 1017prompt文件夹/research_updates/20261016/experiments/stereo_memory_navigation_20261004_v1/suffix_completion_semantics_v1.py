"""Archived current/past eligibility only; no policy modification or motion claim."""
import numpy as np

def suffix_completion_eligibility(current,previous,prior_plan):
 r=dict(eligible=False,reasons=[],endpoint_distance_m=None,remaining_arc_m=None,physical_safety_certified=False,online_validated=False)
 d=current.get('reference_decision',{})
 try:
  clocks=np.array([current['time_s'],previous['time_s'],d['measured_time_s'],prior_plan['time_s']],float)
 except (KeyError,TypeError,ValueError):
  r['reasons']=['missing_clock'];return r
 if not np.isfinite(clocks).all() or not 0<clocks[0]-clocks[1]<=.1+1e-6 or abs(clocks[2]-clocks[0])>1e-6 or abs(clocks[3]-clocks[1])>1e-6 or clocks[3]>=clocks[0]:
  r['reasons']=['nonfinite_or_noncausal_clock'];return r
 if d.get('decision')!='rebuild' or abs(float(d.get('measured_time_s',-1))-current['time_s'])>1e-6:
  r['reasons']=['not_current_rebuild_boundary'];return r
 tr=d.get('state_before',{})
 if tr.get('active_direction')!='forward' or tr.get('pending_direction') is not None or tr.get('failure') is not None or tr.get('braking_state')!='tracking' or d.get('tube_viability_armed_at_boundary'):
  r['reasons']=['not_ordinary_forward'];return r
 g=d.get('guard')
 if not g or g.get('reject_reasons')!=['remaining_arc_under_lookahead'] or not g.get('continuous_clearance_pass') or not g.get('raster_clearance_pass') or g['projection_distance_m']>.5:
  r['reasons']=['other_guard_or_missing'];return r
 if previous['frame']>=current['frame'] or not 0<current['time_s']-previous['time_s']<=.1+1e-6 or prior_plan['frame']!=previous['frame'] or prior_plan['reference_version']!=previous['local_reference_version']:
  r['reasons']=['not_immediately_past_plan'];return r
 w=prior_plan.get('waypoints',[])
 if prior_plan.get('status')!='success' or not w or prior_plan.get('direction_changes')!=0 or any(x['drive_direction']!='forward' for x in w):
  r['reasons']=['not_all_forward_reference'];return r
 path=np.array([[x['x_m'],x['y_m']] for x in w]);old=np.asarray(previous['installed_reference_xy_after_act'],float);suffix=np.asarray(g['suffix_xy'],float);bridge=np.asarray(g['bridge_xy'],float)
 if path.shape!=old.shape or not np.array_equal(path,old) or not np.array_equal(suffix[-1],path[-1]) or not np.isfinite(path).all() or not np.isfinite(suffix).all() or not np.isfinite(bridge).all():
  r['reasons']=['reference_identity_invalid'];return r
 r['endpoint_distance_m']=float(np.linalg.norm(bridge[0]-path[-1]));r['remaining_arc_m']=float(g['remaining_arc_m'])
 if r['endpoint_distance_m']<=.2:r['reasons']=['endpoint_already_within_existing_arrival_tolerance'];return r
 r['eligible']=True;return r
