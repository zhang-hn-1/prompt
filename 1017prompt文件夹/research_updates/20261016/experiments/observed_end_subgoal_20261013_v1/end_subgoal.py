"""Current observed-end exploratory target; no free-space or completed-wall claim."""
import types,time
import numpy as np
from scipy.ndimage import distance_transform_edt,label
import minimum_prefix_transition_planning_v1 as search
from minimum_prefix_transition_consumer_v1 import local_curve_first_direction as original
from execution_domain_four_point_consumer_v2 import planning_clearance
from execution_domain_four_point_source_v2 import supported_curvature,supported_requests
from benchmark.kinodynamic_planning import KinodynamicWaypoint as WP,KinodynamicPlanResult as PR

def select_target(grid,xy,goal,old_target):
    xy,goal,old_target=[np.asarray(x,float) for x in (xy,goal,old_target)]
    if any(x.shape!=(2,) or not np.isfinite(x).all() for x in (xy,goal,old_target)):
        raise ValueError('finite current query required')
    detail=dict(exploratory_unknown=True,observed_free=False,completed_obstacle=False)
    margin=planning_clearance(grid)
    clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
    components,_=label(clearance<margin-1e-8,np.ones((3,3)))
    delta=goal-xy;length=float(np.linalg.norm(delta))
    if length==0:return None,dict(detail,reason='at_goal')
    ray=xy+np.linspace(0,1,max(2,int(np.ceil(length/.05))+1))[:,None]*delta
    cell=grid.cells(ray);inside=((cell>=0)&(cell<np.asarray(grid.labels.shape))).all(axis=1)
    ids=components[tuple(cell[inside].T)];hits=ids[ids>0]
    if len(hits)==0:return None,dict(detail,reason='no_blocking_observed_component')
    actual=np.argwhere((components==hits[0])&(grid.labels==1))
    if len(actual)<3:return None,dict(detail,reason='insufficient_observed_extent')
    points=grid.origin+actual*grid.resolution;mean=points.mean(axis=0)
    eig,vec=np.linalg.eigh((points-mean).T@(points-mean)/len(points))
    if eig[1]<=0 or eig[1]<4*eig[0]:return None,dict(detail,reason='nonelongated_observed_component',eigenvalues=eig.tolist())
    axis=vec[:,1];axis*=1 if axis[np.argmax(np.abs(axis))]>=0 else -1
    along=float((old_target-mean)@axis)
    if abs(along)<1e-12:return None,dict(detail,reason='no_route_end_direction')
    sign=1 if along>0 else -1
    normal=np.array([-axis[1],axis[0]]);side=float((xy-mean)@normal)
    if abs(side)<1e-12:return None,dict(detail,reason='no_observer_side')
    normal*=np.sign(side)
    end=points[int(np.argmax(sign*((points-mean)@axis)))]
    desired=end+sign*axis*.3+normal*margin
    detail.update(axis=axis.tolist(),end_sign=sign,observed_end=end.tolist(),normal=normal.tolist(),desired_target=desired.tolist(),support_cells=len(points),eigenvalues=eig.tolist(),planning_clearance_m=margin)
    if np.linalg.norm(desired-xy)>2.6+1e-8:return None,dict(detail,reason='outside_original_horizon_and_tolerance')
    center=grid.cells(xy[None])[0];lo=np.maximum(center-40,0);hi=np.minimum(center+41,grid.labels.shape)
    c=grid.cells(desired[None])[0];neighbors=np.array([c+[a,b] for a in (-1,0,1) for b in (-1,0,1)])
    valid=((neighbors>=lo)&(neighbors<hi)).all(axis=1);neighbors=neighbors[valid]
    if not len(neighbors):return None,dict(detail,reason='outside_crop')
    world=grid.origin+neighbors*grid.resolution;dist=np.linalg.norm(world-desired,axis=1)
    valid=(clearance[tuple(neighbors.T)]>=margin-1e-8)&(dist<=np.sqrt(2)*grid.resolution+1e-8)&(np.linalg.norm(world-xy,axis=1)<=2.6+1e-8)
    if not valid.any():return None,dict(detail,reason='no_clear_target_rounding')
    indices=np.flatnonzero(valid);j=indices[np.argmin(dist[indices])]
    return world[j],dict(detail,reason='observed_end_exploration',target_cell=neighbors[j].tolist())

def end_curve(grid,xy,yaw,worldgoal,drive_model,*,initial_direction,first_primitive_direction=None):
    base,meta=original(grid,xy,yaw,worldgoal,drive_model,initial_direction=initial_direction,first_primitive_direction=first_primitive_direction)
    if base is None:return base,dict(meta,observed_end=dict(fallback=True,reason='baseline_no_plan'))
    target,detail=select_target(grid,xy,worldgoal,meta['local_target'])
    detail.update(baseline_target=meta['local_target'],fallback=target is None)
    if target is None:return base,dict(meta,observed_end=detail)
    t=time.perf_counter();center=grid.cells(np.asarray(xy)[None])[0];lo=np.maximum(center-40,0);hi=np.minimum(center+41,grid.labels.shape)
    clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
    occ=(clearance<planning_clearance(grid)-1e-8).astype('uint8')[lo[0]:hi[0],lo[1]:hi[1]];origin=grid.origin+lo*grid.resolution
    req=search.KinodynamicPlanRequest(occ,grid.resolution,tuple(origin),tuple(xy),float(yaw),tuple(target),node_budget=3000)
    result=search.plan_kinodynamic_path(req,drive_model,initial_direction=initial_direction,first_primitive_direction=first_primitive_direction,primitive_curvature_source=supported_curvature,primitive_requests=supported_requests())
    p=PR(result.status,tuple(WP(**vars(w)) for w in result.waypoints),result.expanded_nodes,result.path_length_m,result.direction_changes)
    detail.update(search_status=p.status,seconds=time.perf_counter()-t,target=target.tolist(),fallback=p.status!='success' or len(p.waypoints)<2)
    if detail['fallback']:return base,dict(meta,observed_end=detail)
    return p,dict(meta,observed_end=detail,local_target=target.tolist(),local_path_length_m=p.path_length_m,direction_changes=p.direction_changes,expanded_nodes=p.expanded_nodes,status='success')

def with_observed_end(base):
    class ObservedEnd(base):
        def plan_boundary(self,*args,**kwargs):
            method=super().plan_boundary;scope=dict(method.__func__.__globals__)
            if 'local_curve_first_direction' not in scope:raise RuntimeError('expected frozen first-direction boundary')
            scope['local_curve_first_direction']=end_curve
            return types.FunctionType(method.__func__.__code__,scope,closure=method.__func__.__closure__)(self,*args,**kwargs)
    return ObservedEnd
