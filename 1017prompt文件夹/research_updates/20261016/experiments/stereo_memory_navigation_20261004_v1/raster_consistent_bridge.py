"""One current-bridge raster-consistent screen. Not physical prediction/safety."""
import math

def bridge_screen(measurement,now,resolution):
 t=float(measurement['time_s'])
 if not math.isfinite(t) or not math.isfinite(now) or abs(t-now)>1e-5:raise ValueError('current guard clock before payload')
 if not math.isfinite(resolution) or resolution<=0:raise ValueError('finite positive resolution')
 g=measurement['guard'];h=resolution*math.sqrt(2)/2;bound=.6-h;body_bound=.6-2*h
 original=bool(g is not None and g['retention_necessary_screen_pass'])
 result=dict(original_retain=original,candidate_retain=original,bridge_bound_m=bound,future_suffix_bound_m=.6,raster_bound_m=.6,body_cell_clearance_lower_bound_m=body_bound,frozen_physical_radius_m=.4,alias_only_current_bridge=False,nominal_geometry_only=True,certified_safe=False)
 if original or g is None:return result
 values=[g.get(k) for k in ['suffix_clearance_m','bridge_clearance_m','projection_distance_m','remaining_arc_m']]
 if any(v is None or not math.isfinite(v) for v in values):return result
 suffix,bridge,projection,remaining=values
 valid=(body_bound>=.4-1e-9 and g['raster_clearance_pass'] is True and suffix>=.6-1e-8 and bridge>=bound-1e-8 and projection<=.5 and remaining>=.5 and set(g['reject_reasons'])=={'continuous_observed_hazard_clearance'})
 result.update(candidate_retain=bool(valid),alias_only_current_bridge=bool(valid));return result
