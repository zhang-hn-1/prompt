"""Immutable context after exactly one actual emission, without another policy act.
Tracker raw/protocol request and final outer issued request are distinct.
Smoothed/applied targets are unavailable here, not future prediction inputs.
"""
from dataclasses import dataclass,asdict
import math
import numpy as np

@dataclass(frozen=True)
class CurrentRequestContext:
 frame:int
 time_s:float
 emission_count:int
 records_reset_during_act:bool
 emission_source:str
 raw_tracker_component:tuple
 protocol_tracker_component:tuple
 issued_request:tuple
 geometry_computed_this_call:bool
 normal_tracking_this_call:bool
 requested_curvature_1_m:object
 reference_direction:object
 requested_direction:str
 active_direction:str
 pending_direction:object
 braking_state:str
 measured_world_xy_velocity:tuple
 measured_body_forward_velocity_m_s:float
 measured_main_joint_velocity_rad_s:float
 applied_target:object=None
 scope:str='post_single_act_current_request_only_not_execution_or_forecast'

def build_request_context(*,frame,time_s,observation,tracker,before_emission_count,issued_request,before_emission_records=None):
 if isinstance(frame,bool) or not isinstance(frame,(int,np.integer)) or frame<0:raise ValueError('nonnegative capture frame')
 if isinstance(time_s,bool) or not math.isfinite(float(time_s)):raise ValueError('finite current clock')
 if isinstance(before_emission_count,bool) or not isinstance(before_emission_count,int) or before_emission_count<0:raise ValueError('emission count token')
 records=tracker._tracking_records
 reset=before_emission_records is not None and records is not before_emission_records
 if len(records)!=(1 if reset else before_emission_count+1):raise ValueError('exactly one new emission required')
 audit=dict(records[-1]);issued=np.asarray(issued_request,float)
 xy=np.asarray(observation.position_world_xy,float);proprio=np.asarray(observation.proprio_history,float);yaw=float(observation.yaw_world_rad)
 if xy.shape!=(2,) or proprio.ndim!=2 or len(proprio)<1 or proprio.shape[1]!=17 or not np.isfinite(np.r_[xy,proprio.ravel(),yaw]).all():raise ValueError('finite current observation')
 signature=np.r_[xy,yaw,proprio[-1,:2]]
 recorded=np.asarray([audit.get(k,np.nan) for k in ['position_world_x_m','position_world_y_m','root_heading_rad','velocity_world_x_m_s','velocity_world_y_m_s']],float)
 if not np.array_equal(signature,recorded):raise ValueError('emission audit must match current observation')
 keys=['policy_raw_main_command_rad_s','policy_raw_auxiliary_target_rad','policy_protocol_main_command_rad_s','policy_protocol_auxiliary_target_rad']
 values=np.array([audit.get(k,np.nan) for k in keys],float)
 if not np.isfinite(values).all() or issued.ndim!=1 or len(issued)!=int(tracker.protocol.action_block_dim) or not np.isfinite(issued).all():raise ValueError('finite current request layers')
 valid=bool(audit.get('valid',False));curvature=audit.get('requested_curvature_1_m') if valid else None
 if curvature is not None and not math.isfinite(float(curvature)):raise ValueError('finite current geometry')
 normal=valid and curvature is not None and tracker._braking_state=='tracking' and tracker._pending_drive_direction is None and bool(np.any(issued[::2]!=0))
 signs=set(np.sign(issued[::2]).tolist());requested='zero' if signs=={0.} else 'forward' if signs<={0.,1.} else 'reverse' if signs<={0.,-1.} else 'mixed_block'
 reference=getattr(tracker,'_reference',None);index=getattr(tracker,'_path_index',None)
 direction=None if reference is None or index is None else reference[index].drive_direction
 return CurrentRequestContext(int(frame),float(time_s),len(records),reset,str(audit.get('source','unknown')),tuple(values[:2]),tuple(values[2:]),tuple(issued),valid,normal,None if curvature is None else float(curvature),direction,requested,tracker._active_drive_direction,tracker._pending_drive_direction,tracker._braking_state,tuple(proprio[-1,:2]),float(proprio[-1,:2]@np.array([math.cos(yaw),math.sin(yaw)])),float(proprio[-1,12]))

def with_current_request_context(base):
 class RequestContextPolicy(base):
  def reset(self,context):
   super().reset(context);self.current_request_context=None;self.request_context_records=[]
  def act(self,observation,goal):
   packet=self._packet
   if packet is None:raise ValueError('current packet required before single act')
   previous_records=self.tracker._tracking_records;count=len(previous_records);frame=int(packet['frame']);t=float(packet['time_s'])
   action=super().act(observation,goal)
   context=build_request_context(frame=frame,time_s=t,observation=observation,tracker=self.tracker,before_emission_count=count,issued_request=action,before_emission_records=previous_records)
   self.current_request_context=context;self.request_context_records.append(context)
   self.records[-1]['current_request_context']=asdict(context)
   return action
 return RequestContextPolicy
