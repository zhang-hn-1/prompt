"""Retrospective current-state geometry checks, never policy/model inputs."""
import numpy as np
from scipy.spatial.transform import Rotation
from forward_course_geometry_v1 import select_forward_geometry

def classify(*,event,changed,isolated,prefix,performance):
 if not (event and changed and isolated and prefix):return 'unassessed_event_or_interface'
 return 'conditional_knownpose_TaskH_accept' if performance else 'measured_navigation_reject'

def verify_geometry(row,state,history,*,enabled):
 decision=row['forward_course_geometry']
 if not decision.get('valid'):
  assert not decision['used'];return
 snap=decision['composition_input'];chain=row['current_tracking_chain']
 now=float(row['time_s']);assert np.isfinite(now)
 assert abs(now-float(state['time_s']))<1e-7 and abs(now-snap['time_s'])<1e-7
 clocks=np.asarray([h[0] for h in history],float)
 assert len(clocks)>0 and np.isfinite(clocks).all() and abs(clocks[-1]-now)<1e-7
 assert np.allclose(np.diff(clocks),.1,atol=1e-6,rtol=0)
 xy=np.asarray(state['position'][:2],float);q=np.asarray(state['quaternion'],float)
 assert np.isfinite(xy).all() and q.shape==(4,) and np.isfinite(q).all() and np.linalg.norm(q)>1e-9
 yaw=float(Rotation.from_quat(q[[1,2,3,0]]).as_euler('xyz')[2])
 assert abs(np.arctan2(np.sin(yaw-snap['root_yaw_rad']),np.cos(yaw-snap['root_yaw_rad'])))<1e-6
 assert abs(np.arctan2(np.sin(yaw-chain['root_heading_rad']),np.cos(yaw-chain['root_heading_rad'])))<1e-6
 np.testing.assert_allclose(xy,snap['position_world_xy'],atol=1e-6,rtol=0)
 reference=row['current_reference_waypoints_after_act'];projection=int(chain['projection_index'])
 assert 0<=projection<len(reference)
 refxy=np.asarray([[w['x_m'],w['y_m']] for w in reference],float)
 # Current factory uses the frozen tracker default .5m, not an online asserted lookahead.
 speed=float(np.linalg.norm(history[-1][1]));lookahead=max(.5,float(np.clip(.45+.8*speed,.45,.9)))
 assert abs(chain['lookahead_requested_m']-lookahead)<1e-7
 target_index=projection;accumulated=0.;direction=reference[projection]['drive_direction']
 while target_index+1<len(reference):
  if reference[target_index+1]['drive_direction']!=direction:break
  accumulated+=float(np.linalg.norm(refxy[target_index+1]-refxy[target_index]));target_index+=1
  if accumulated>=lookahead:break
 assert target_index==chain['lookahead_index'] and abs(accumulated-chain['lookahead_accumulated_m'])<1e-7
 target=np.asarray([chain['lookahead_x_m'],chain['lookahead_y_m']]);np.testing.assert_allclose(target,refxy[target_index],atol=1e-8,rtol=0)
 np.testing.assert_allclose(target,snap['lookahead_world_xy'],atol=1e-8,rtol=0)
 delta=target-xy;den=float(delta@delta)
 parent=0. if den<=1e-9 else 2*(-np.sin(yaw)*delta[0]+np.cos(yaw)*delta[1])/den
 assert abs(parent-chain['geometric_pure_pursuit_curvature_1_m'])<1e-6
 ref=row['current_reference_waypoints_after_act'][chain['projection_index']]
 assert ref['drive_direction']==snap['reference_direction'] and ref['main_command_rad_s']==snap['nominal_main_rad_s']
 assert decision['enabled']==enabled
 if enabled and abs(snap['nominal_main_rad_s'])>1e-12:
  selected,expected=select_forward_geometry(parent,delta,now=now,times=clocks,velocity=[h[1] for h in history],main_velocity=[h[2] for h in history],active=snap['active'],pending=snap['pending'],braking=snap['braking'],reference_direction=snap['reference_direction'])
  assert decision['used']==expected['used']
  if expected['used']:
   assert abs(expected['course_heading_rad']-decision['course_heading_rad'])<1e-8
   assert abs(selected-decision['selected_curvature'])<1e-6
 else:
  selected=parent;assert not decision['used']
 assert abs(selected-chain['requested_curvature_1_m'])<1e-6


def verify_current_proprio(prop,physics,now):
 times=np.asarray(physics['simulation_time_s']);matches=np.flatnonzero(np.isclose(times,now,atol=1e-6,rtol=0))
 assert len(matches)==1
 i=int(matches[0]);np.testing.assert_array_equal(np.asarray(prop[:2],np.float32),np.asarray(physics['linear_velocity_world_m_s'][i,:2],np.float32))
 assert np.float32(prop[12])==np.float32(physics['joint_velocity_rad_s'][i,0])
