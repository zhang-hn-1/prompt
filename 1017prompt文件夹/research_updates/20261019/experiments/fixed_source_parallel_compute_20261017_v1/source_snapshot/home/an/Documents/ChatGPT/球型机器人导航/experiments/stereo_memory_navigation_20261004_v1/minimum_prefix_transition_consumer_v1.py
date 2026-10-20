"""One planner-domain experiment; old boundary/retention/act/PID otherwise unchanged."""
import types
import numpy as np
import minimum_prefix_transition_planning_v1 as search
import execution_domain_four_point_consumer_v2 as old
from guarded_reference import boundary_state

def measured_first_direction(observation,state):
    p=np.asarray(observation.proprio_history,float)
    if p.ndim!=2 or p.shape[1]!=17 or not np.isfinite(p).all():
        raise ValueError('finite current proprio schema17')
    yaw=float(observation.yaw_world_rad)
    if not np.isfinite(yaw):raise ValueError('finite current yaw')
    heading=np.array([np.cos(yaw),np.sin(yaw)])
    eligible=(state['active_direction']=='forward' and state['pending_direction'] is None
              and state['braking_state']=='tracking' and p[-1,12]>.1
              and np.linalg.norm(p[-1,:2])>.05 and p[-1,:2]@heading>0)
    if eligible:
        from cusp_coast_acquisition_v1 import CONFIG
        from roll_pid import bank_from_quaternion
        bank=float(bank_from_quaternion(np.asarray(observation.quaternion_world_wxyz_history[-1],float)))
        lo,hi=CONFIG['speed_domain_m_s'];jlo,jhi=CONFIG['main_joint_velocity_domain_rad_s']
        speed=float(np.linalg.norm(p[-1,:2]))
        eligible=(lo-1e-8<=speed<=hi+1e-8 and jlo-1e-8<=p[-1,12]<=jhi+1e-8
                  and abs(bank)<=CONFIG['bank_abs_max_rad']+1e-8)
    return 'forward' if eligible else None

def local_curve_first_direction(*args,first_primitive_direction=None,**kwargs):
    def planner(req,drive,**kw):
        return search.plan_kinodynamic_path(search.KinodynamicPlanRequest(**vars(req)),drive,
            first_primitive_direction=first_primitive_direction,**kw)
    scope=dict(old.local_curve_for_direction_tube.__globals__,plan_kinodynamic_path=planner)
    fn=types.FunctionType(old.local_curve_for_direction_tube.__code__,scope)
    return fn(*args,**kwargs)

def with_first_primitive_direction(base):
    class FirstPrimitiveDirection(base):
        def reset(self,context):
            super().reset(context);self.current_first_direction=None;self.first_direction_query=None
        def act(self,observation,goal):
            self.current_first_direction=measured_first_direction(observation,boundary_state(self.tracker)) if self.tracker._context is not None else None
            self.first_direction_query=None
            action=super().act(observation,goal)
            self.records[-1]['first_primitive_direction_query']=self.first_direction_query
            return action
        def plan_boundary(self,*args,**kwargs):
            # Clone the inherited old boundary's namespace for this call only. No global monkeypatch,
            # no second stateful act, and no rewriting its retention or installation transaction.
            method=super().plan_boundary
            scope=dict(method.__func__.__globals__)
            def local(*a,**kw):
                self.first_direction_query=dict(direction=self.current_first_direction,
                    measured_state=boundary_state(self.tracker),first_expansion_only=False,minimum_initial_run_primitives=2 if self.current_first_direction else 0,minimum_nominal_initial_arc_m=.6 if self.current_first_direction else 0.,physical_stopping_bound_certified=False,
                    later_reverse_allowed=True,current_and_past_only=True)
                plan,meta=local_curve_first_direction(*a,first_primitive_direction=self.current_first_direction,**kw)
                self.first_direction_query.update(status=meta['status'],expanded_nodes=meta.get('expanded_nodes'),
                    first_queried_direction=None if plan is None else plan.waypoints[0].drive_direction)
                return plan,meta
            if 'local_curve_for_direction_tube' not in scope:raise RuntimeError('unsupported inherited boundary identity')
            scope['local_curve_for_direction_tube']=local
            fn=types.FunctionType(method.__func__.__code__,scope,closure=method.__func__.__closure__)
            return fn(self,*args,**kwargs)
    return FirstPrimitiveDirection
