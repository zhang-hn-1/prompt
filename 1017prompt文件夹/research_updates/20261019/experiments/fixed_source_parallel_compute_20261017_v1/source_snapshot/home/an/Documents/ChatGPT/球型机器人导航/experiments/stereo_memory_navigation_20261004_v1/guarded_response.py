"""Guarded finite measured response consumption, not a general motion model.

Historical XY is a model table, current state is measured; no future feedback
from the consuming run is an input. Entire archived turn+stop is indivisible.
"""
import copy
import numpy as np
from primitive_planner import assess_support
from execution_snapshot import ExecutionClone

# Frozen before fresh run. Narrow applicability tolerances, not safety bounds.
TOL=dict(linear_local=.015,angular_local=.04,joint_position=.05,joint_velocity=.05,
    tilt_angle_rad=np.deg2rad(2),pid_integral=.01,pid_previous_bank=np.deg2rad(2),pid_rate=.04,inner_action=.005)
def yaw_q(q):
    w,x,y,z=np.asarray(q,float)/np.linalg.norm(q)
    return float(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
def physical_state(position,q,linear,angular,joints,joint_vel):
    q=np.asarray(q,float);q=q/np.linalg.norm(q);yaw=yaw_q(q);c,s=np.cos(yaw),np.sin(yaw)
    R=np.array([[c,s,0],[-s,c,0],[0,0,1]])
    w,x,y,z=q;ch,sh=np.cos(yaw/2),np.sin(yaw/2)
    tilt=np.array([ch*w+sh*z,ch*x+sh*y,ch*y-sh*x,ch*z-sh*w])
    return dict(position=np.asarray(position,float).tolist(),yaw=yaw,tilt_q=tilt.tolist(),linear_local=(R@linear).tolist(),angular_local=(R@angular).tolist(),joint_position=np.asarray(joints,float).tolist(),joint_velocity=np.asarray(joint_vel,float).tolist())
def make_entry(name,state,execution,xy,requests,times):
    return dict(name=name,state=copy.deepcopy(state),execution=copy.deepcopy(execution),xy=np.asarray(xy,float).copy(),requests=np.asarray(requests,np.float32).copy(),times=np.asarray(times,float).copy())
def applicability(current,execution,entry):
    reasons=[];delta={}
    try:
        ExecutionClone.from_snapshot(execution) # completeness & validated vector sizes
        reference=entry['execution']
        for key in ('schema','protocol','inner_config','pid_config'):
            if execution[key]!=reference[key]:reasons.append('identity_'+key)
        for key in ('linear_local','angular_local','joint_position','joint_velocity'):
            d=np.linalg.norm(np.asarray(current[key])-entry['state'][key]);delta[key]=float(d)
            if not np.isfinite(d) or d>TOL[key]:reasons.append(key)
        qa=np.asarray(current['tilt_q'],float);qb=np.asarray(entry['state']['tilt_q'],float)
        angle=2*np.arccos(np.clip(abs(np.dot(qa/np.linalg.norm(qa),qb/np.linalg.norm(qb))),0,1));delta['tilt_angle_rad']=float(angle)
        if not np.isfinite(angle) or angle>TOL['tilt_angle_rad']:reasons.append('tilt')
        if execution['blocked']!=reference['blocked']:reasons.append('supervisor_previous_blocked')
        for key,limit in [('integral_output','pid_integral'),('previous_bank','pid_previous_bank'),('rate','pid_rate')]:
            a,b=execution['pid'][key],reference['pid'][key]
            d=float('inf') if ((a is None)!=(b is None)) else (0. if a is None else abs(a-b))
            delta['pid_'+key]=d
            if not np.isfinite(d) or d>TOL[limit]:reasons.append('pid_'+key)
        for key in ('current_action','last_smoothed_target'):
            d=float(np.max(np.abs(np.asarray(execution['inner'][key])-reference['inner'][key])));delta[key]=d
            if not np.isfinite(d) or d>TOL['inner_action']:reasons.append('inner_'+key)
    except (KeyError,TypeError,ValueError):reasons.append('incomplete_execution_or_measurement')
    return dict(applicable=not reasons,reasons=reasons,deltas=delta)

class GuardedResponseConsumer:
    def __init__(self,entries):self.entries=entries
    def select(self,current,execution,goal,labels,origin,resolution):
        checked=[];options=[]
        for e in self.entries:
            gate=applicability(current,execution,e);row=dict(name=e['name'],gate=gate);checked.append(row)
            if not gate['applicable']:continue
            yaw=current['yaw'];c,s=np.cos(yaw),np.sin(yaw);R=np.array([[c,-s],[s,c]])
            xy=e['xy']@R.T+np.asarray(current['position'])[:2]
            support=assess_support(xy,labels,origin,resolution,.5)
            row['support']=support
            if support['occupied_cells']:continue
            score=float(np.linalg.norm(xy[-1]-goal));row['endpoint_goal_distance_m']=score
            options.append((score,e,xy,support))
        if not options:
            any_gate=any(r['gate']['applicable'] for r in checked)
            return dict(selected=None,status='recorded_hazard' if any_gate else 'unsupported_initial_state',checked=checked,full_3d_certified=False)
        _,e,xy,support=min(options,key=lambda v:(v[0],v[1]['name']))
        return dict(selected=e['name'],status='unknown_risk' if support['unknown_cells'] else 'observed_2p5d_support',checked=checked,
            world_xy=xy.copy(),requests=e['requests'].copy(),times=e['times'].copy(),support=support,full_3d_certified=False,
            scope='one indivisible measured6s turn + matched6s stopping segment; no arbitrary replanning or stopped forever claim')
