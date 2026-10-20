import numpy as np
from scipy.spatial.transform import Rotation
from high_main_turn_measurement_v1 import measured_arm

def fixture(n=300,collision=False,speed=.4):
 t=np.arange(5*n+1)*.02;ang=t*speed/2
 xy=np.c_[2*np.sin(ang),2*(1-np.cos(ang))]
 yaw=ang;qq=Rotation.from_euler('ZYX',np.c_[yaw,np.zeros(len(t)),np.full(len(t),-np.pi/12)]).as_quat()[:,[3,0,1,2]]
 vel=np.c_[speed*np.cos(yaw),speed*np.sin(yaw),np.zeros(len(t))];vel[t>=22]=0
 p=dict(simulation_time_s=t,position_world_xy=xy,root_quaternion_world_wxyz=qq,linear_velocity_world_m_s=vel,yaw_world_rad=yaw)
 states=[dict(time_s=float(t[i]),velocity=vel[i].tolist()) for i in range(0,len(t),5)]
 contacts=[dict(simulation_time_s=float(x),available=True,contact=collision and i==5*n) for i,x in enumerate(t)]+[dict(simulation_time_s=i*.1,available=True,contact=False) for i in range(1,n+1)]
 return states,contacts,p

def test_actual_circle_curvature_bank_speed_and_terminal_feedback():
 s,c,p=fixture();r=measured_arm(dict(status='finished',child_returncode=0),dict(complete=True,actions=300,captures=301),s,c,p,300)
 assert r['classification']=='complete_response'
 assert abs(r['window']['curvature_body_1_m']-.5)<.001
 assert abs(r['window']['actual_bank_deg']+15)<.001
 assert abs(r['actual_mean_speed_m_s']-.4)<1e-6 and r['terminal_stop']['last_count']>=5
 assert r['window']['accepted']

def test_partial_physical_collision_and_interrupt_are_distinct():
 s,c,p=fixture(10,True)
 for status,expected in [(dict(status='finished',child_returncode=0),'measured_collision_reject'),(dict(status='terminated',child_returncode=-15),'unassessed_execution')]:
  r=measured_arm(status,dict(complete=False,actions=10,captures=11),s,c,p,10)
  assert r['classification']==expected

def test_missing_contact_tick_is_not_measured_response():
 s,c,p=fixture();r=measured_arm(dict(status='finished',child_returncode=0),dict(complete=True,actions=300,captures=301),s,c[:-1],p,300)
 assert r['classification']=='unassessed_evidence'
