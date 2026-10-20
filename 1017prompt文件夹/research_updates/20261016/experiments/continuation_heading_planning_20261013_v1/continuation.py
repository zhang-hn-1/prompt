"""Require local terminal motion to agree with current outgoing grid guidance."""
import time,types,numpy as np
from scipy.ndimage import distance_transform_edt
import continuation_search as search
from minimum_prefix_transition_consumer_v1 import local_curve_first_direction as original
from execution_domain_four_point_consumer_v2 import planning_clearance
from execution_domain_four_point_source_v2 import supported_curvature,supported_requests
from benchmark.kinodynamic_planning import KinodynamicWaypoint as WP,KinodynamicPlanResult as PR

def outgoing(meta):
    route=np.asarray(meta['route_xy'],float);arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
    i=min(len(route)-1,int(np.searchsorted(arc,2.4)))
    if i==len(route)-1:return None
    j=min(len(route)-1,int(np.searchsorted(arc,arc[i]+.3)))
    v=route[j]-route[i]
    return v/np.linalg.norm(v)

def projection(plan,tangent):
    if plan is None or tangent is None:return None
    w=plan.waypoints[-1];sign=1 if w.drive_direction=='forward' else -1
    return float(sign*np.array([np.cos(w.yaw_rad),np.sin(w.yaw_rad)])@tangent)

def continuation_curve(grid,xy,yaw,worldgoal,drive_model,*,initial_direction,first_primitive_direction=None):
    b,meta=original(grid,xy,yaw,worldgoal,drive_model,initial_direction=initial_direction,first_primitive_direction=first_primitive_direction)
    if 'route_xy' not in meta:return b,dict(meta,terminal_continuation=dict(status='no_grid_route',fallback=True))
    tangent=outgoing(meta)
    if tangent is None:return b,dict(meta,terminal_continuation=dict(status='final_goal_no_outgoing_constraint',fallback=True))
    started=time.perf_counter();center=grid.cells(np.asarray(xy)[None])[0];lo=np.maximum(center-40,0);hi=np.minimum(center+41,grid.labels.shape)
    clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
    occupied=(clearance<planning_clearance(grid)-1e-8).astype('uint8')[lo[0]:hi[0],lo[1]:hi[1]];origin=grid.origin+lo*grid.resolution
    req=search.KinodynamicPlanRequest(occupied,grid.resolution,tuple(origin),tuple(xy),float(yaw),tuple(meta['local_target']),node_budget=3000)
    q=search.plan_kinodynamic_path(req,drive_model,initial_direction=initial_direction,first_primitive_direction=first_primitive_direction,terminal_motion_vector=tuple(tangent),primitive_curvature_source=supported_curvature,primitive_requests=supported_requests())
    p=PR(q.status,tuple(WP(**vars(w)) for w in q.waypoints),q.expanded_nodes,q.path_length_m,q.direction_changes)
    detail=dict(status=p.status,expanded_nodes=p.expanded_nodes,seconds=time.perf_counter()-started,outgoing_tangent=tangent.tolist(),baseline_terminal_projection=projection(b,tangent),candidate_terminal_projection=projection(p,tangent) if p.status=='success' else None,fallback=p.status!='success' or len(p.waypoints)<2,first_direction=first_primitive_direction,baseline_path_length_m=None if b is None else b.path_length_m,baseline_direction_changes=None if b is None else b.direction_changes,baseline_waypoints=[] if b is None else [vars(w) for w in b.waypoints])
    if detail['fallback']:return b,dict(meta,terminal_continuation=detail)
    return p,dict(meta,terminal_continuation=detail,local_path_length_m=p.path_length_m,direction_changes=p.direction_changes,expanded_nodes=p.expanded_nodes,status='success')

def with_continuation_heading(base):
    class ContinuationHeading(base):
        def plan_boundary(self,*args,**kwargs):
            method=super().plan_boundary;scope=dict(method.__func__.__globals__)
            if 'local_curve_first_direction' not in scope:raise RuntimeError('expected frozen first-direction boundary')
            scope['local_curve_first_direction']=continuation_curve
            return types.FunctionType(method.__func__.__code__,scope,closure=method.__func__.__closure__)(self,*args,**kwargs)
    return ContinuationHeading
