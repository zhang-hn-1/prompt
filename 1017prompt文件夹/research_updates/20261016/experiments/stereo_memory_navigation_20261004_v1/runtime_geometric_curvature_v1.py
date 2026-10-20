"""Reuse frozen Round5 geometry definitions on new closed runtime traces only."""
import hashlib,json
import numpy as np
from scripts.analyze_round5_motion_transitions import measure_motion_quantities

def measure(t,p,v,q,omega):
 return measure_motion_quantities(time_s=t,position_xy_m=p,linear_velocity_world_xy_m_s=v,root_quaternion_world_wxyz=q,angular_velocity_world_rad_s=omega,minimum_planar_speed_m_s=.05)

def windows(rows):
 groups=[];start=None;previous=None
 for r in rows:
  c=r.get('current_tracking_chain',{});s=r.get('tracker_state_after_act',{});key=None
  if c.get('valid') and c.get('fresh_tracker_record') and c.get('source')=='tracking' and s.get('pending_direction') is None and s.get('braking_state')=='tracking' and c.get('reference_drive_direction')==s.get('active_direction'):
   ref=r['current_reference_waypoints_after_act'];w=ref[c['projection_index']]
   key=(hashlib.sha256(json.dumps(ref,sort_keys=True).encode()).hexdigest(),w['drive_direction'],w['main_command_rad_s'],w['auxiliary_angle_rad'])
  if key!=previous:
   if start is not None:
    for a in range(start,r['frame'],30):groups.append((a,min(a+30,r['frame']),previous))
   start=r['frame'] if key is not None else None;previous=key
 if start is not None:
  for a in range(start,len(rows),30):groups.append((a,min(a+30,len(rows)),previous))
 return groups
