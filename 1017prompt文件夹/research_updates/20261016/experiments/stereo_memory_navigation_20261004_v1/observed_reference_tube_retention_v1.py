"""One current observed-map containment consumer; no future physical safety claim."""
import numpy as np
from scipy.ndimage import distance_transform_edt
from route_geometry_audit import hazard_clearance

def observed_tube_retention_screen(grid,guard):
 res=float(grid.resolution)
 if abs(res-.1)>1e-9:raise ValueError('frozen .1m grid only')
 error=float(guard['projection_distance_m'])
 if not np.isfinite(error) or error<0:raise ValueError('finite nonnegative current projection')
 pieces={};parts={};origin=np.asarray(grid.origin,float);labels=np.asarray(grid.labels)
 if labels.ndim!=2 or not np.isin(labels,[-1,0,1]).all() or origin.shape!=(2,) or not np.isfinite(origin).all():raise ValueError('finite observed grid')
 field=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*res
 for key in ['bridge_xy','suffix_xy']:
  q=np.asarray(guard[key],float)
  if q.ndim!=2 or q.shape[1]!=2 or len(q)<1 or not np.isfinite(q).all():raise ValueError('finite current geometry')
  pieces[key]=q;chunks=[q[:1]]
  for a,b in zip(q[:-1],q[1:]):chunks.append(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)))
  samples=np.vstack(chunks);cells=np.rint((samples-origin)/res).astype(int);inside=(cells>=0).all(1)&(cells<np.array(labels.shape)).all(1)
  parts[key]=float(field[tuple(cells.T)].min()) if inside.all() else None
 bridge=pieces['bridge_xy'];suffix=pieces['suffix_xy']
 if len(bridge)>2 or not np.allclose(bridge[-1],suffix[0],atol=1e-10,rtol=0) or abs(np.linalg.norm(bridge[0]-bridge[-1])-error)>1e-8:raise ValueError('same measured direct bridge and projection required')
 margin=.6+.2+res*np.sqrt(2)/2
 old=all(v is not None and v>=margin-1e-8 for v in parts.values())
 hazards=origin+np.argwhere(labels==1)*res
 continuous=hazard_clearance(suffix,hazards)
 # Triangle inequality: every bridge point lies in the .2 ball at suffix start;
 # every .2 perturbation of the whole suffix retains >=.6 clearance to cell centres.
 # This is observed-map geometry, not prediction that physics stays in this tube.
 suffixpass=parts['suffix_xy'] is not None and parts['suffix_xy']>=margin-1e-8
 contained=error<=.2+1e-8 and continuous is not None and continuous-.2>=.6-1e-8
 eligible=bool(guard['retention_necessary_screen_pass'])
 exception=eligible and not old and suffixpass and contained and parts['bridge_xy'] is not None
 return dict(**{'pass':bool(eligible and (old or exception))},old_combined_pass=old,exception_used=bool(exception),suffix_raster_clearance_m=parts['suffix_xy'],bridge_raster_clearance_m=parts['bridge_xy'],existing_nominal_planning_clearance_m=margin,suffix_continuous_clearance_m=continuous,current_projection_distance_m=error,existing_tracking_allowance_m=.2,existing_base_observed_clearance_m=.6,geometry_relative_observed_cell_centres_only=True,future_tracking_bound_certified=False,stop_sweep_certified=False,unknown_free=False)

def with_observed_reference_tube_retention(base):
 from guarded_reference import boundary_state
 from reference_continuity_audit import reference_geometry_status
 from execution_domain_four_point_consumer_v2 import local_curve_for_direction_tube
 from depth_local_curve import install_local_reference
 class ObservedTubeRetention(base):
  def plan_boundary(self,xy,yaw,worldgoal,t):
   before=boundary_state(self.tracker);guard=None
   if self.plan is not None:guard=reference_geometry_status(xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)
   keep=guard is not None and guard['retention_necessary_screen_pass'];self.last_tube_viability_screen=None
   if keep and self.tube_viability_armed:
    self.last_tube_viability_screen=observed_tube_retention_screen(self.grid,guard);keep=keep and self.last_tube_viability_screen['pass']
   initial=None
   if not keep:
    initial=None if self.tracker._context is None else before['active_direction']
    if initial not in (None,'forward','reverse'):raise ValueError('valid accepted direction required')
    self.plan,self.plan_meta=local_curve_for_direction_tube(self.grid,xy,yaw,worldgoal,self.tracker.drive_model,initial_direction=initial)
    self.local_reference_version+=1
    if self.plan is not None:install_local_reference(self.tracker,self.plan)
    self.reference_installed_time=t if self.plan is not None else None
   self.last_plan=t
   decision=dict(decision='retain' if keep else 'rebuild',measured_time_s=t,guard=guard,initial_or_no_route=guard is None,state_before=before,state_after=boundary_state(self.tracker),certified_safe=False,direction_cost_input=initial,tube_viability_armed_at_boundary=self.tube_viability_armed,tube_viability_screen=self.last_tube_viability_screen)
   self.reference_decision=decision;return not keep,decision
 return ObservedTubeRetention
