"""Current observed-map global kinematic route; eight-primitive execution prefix."""
import time,types,dataclasses
import numpy as np
from scipy.ndimage import distance_transform_edt
import minimum_prefix_transition_planning_v1 as search
from minimum_prefix_transition_consumer_v1 import local_curve_first_direction as original
from execution_domain_four_point_consumer_v2 import planning_clearance
from execution_domain_four_point_source_v2 import supported_curvature,supported_requests
from benchmark.kinodynamic_planning import KinodynamicWaypoint as WP,KinodynamicPlanResult as PR

def prefix_plan(full):
    points=[WP(**vars(w)) for w in full.waypoints[:9]]
    if len(points)<2:raise ValueError('nonempty primitive prefix required')
    points[-1]=dataclasses.replace(points[-1],main_command_rad_s=0.,auxiliary_angle_rad=0.,drive_direction=points[-2].drive_direction)
    changes=sum(a.drive_direction!=b.drive_direction for a,b in zip(points[:-2],points[1:-1]))
    return PR('success',tuple(points),full.expanded_nodes,(len(points)-1)*.3,changes)

def global_curve(grid,xy,yaw,worldgoal,drive_model,*,initial_direction,first_primitive_direction=None):
    # Exactly one inherited query maintains its causal grid bookkeeping/fallback.
    baseline,meta=original(grid,xy,yaw,worldgoal,drive_model,initial_direction=initial_direction,first_primitive_direction=first_primitive_direction)
    started=time.perf_counter()
    clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
    occupied=(clearance<planning_clearance(grid)-1e-8).astype(np.uint8)
    req=search.KinodynamicPlanRequest(occupied,grid.resolution,tuple(grid.origin),tuple(xy),float(yaw),tuple(worldgoal),node_budget=3000)
    full=search.plan_kinodynamic_path(req,drive_model,initial_direction=initial_direction,first_primitive_direction=first_primitive_direction,primitive_curvature_source=supported_curvature,primitive_requests=supported_requests())
    detail=dict(status=full.status,expanded_nodes=full.expanded_nodes,seconds=time.perf_counter()-started,worldgoal=list(worldgoal),first_direction=first_primitive_direction,full_path_length_m=full.path_length_m,full_direction_changes=full.direction_changes,full_waypoints=[vars(w) for w in full.waypoints],fallback=full.status!='success' or len(full.waypoints)<2,nominal_only=True,truth_online=False)
    if detail['fallback']:
        return baseline,dict(meta,global_kinematic=detail)
    selected=prefix_plan(full)
    result=dict(meta,global_kinematic=detail,baseline_plan_meta=meta,status='success',local_target=[selected.waypoints[-1].x_m,selected.waypoints[-1].y_m],local_path_length_m=selected.path_length_m,direction_changes=selected.direction_changes,expanded_nodes=selected.expanded_nodes,search_origin=list(grid.origin),search_shape=list(occupied.shape),unknown_fraction=1.,certified_safe=False)
    return selected,result

def with_global_guidance(base):
    class GlobalGuidance(base):
        def plan_boundary(self,*args,**kwargs):
            method=super().plan_boundary
            scope=dict(method.__func__.__globals__)
            if 'local_curve_first_direction' not in scope:raise RuntimeError('expected frozen first-direction boundary')
            scope['local_curve_first_direction']=global_curve
            fn=types.FunctionType(method.__func__.__code__,scope,closure=method.__func__.__closure__)
            return fn(self,*args,**kwargs)
    return GlobalGuidance
