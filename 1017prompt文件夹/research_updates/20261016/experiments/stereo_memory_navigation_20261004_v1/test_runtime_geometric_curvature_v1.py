import numpy as np
import pytest

def test_circle_course_curvature_is_distinct_from_world_z_rate():
 from runtime_geometric_curvature_v1 import measure
 t=np.arange(0,3.02,.02);theta=.2*t;pos=np.c_[np.cos(theta),np.sin(theta)];v=np.c_[-.2*np.sin(theta),.2*np.cos(theta)];q=np.tile([1.,0,0,0],(len(t),1));omega=np.zeros((len(t),3));x=measure(t,pos,v,q,omega)
 assert np.median(x['course_curvature_1_m'])==pytest.approx(1.,abs=1e-6)
 assert np.max(abs(x['world_z_angular_velocity_rad_s']))==0
 assert np.median(x['three_point_curvature_1_m'][1:-1])==pytest.approx(1.,abs=1e-6)
 y=measure(t,pos[::-1],-v[::-1],q,omega)
 assert np.median(y['course_curvature_1_m'])==pytest.approx(-1.,abs=1e-6)
 z=measure(t,pos,np.zeros_like(v),q,omega)
 assert not np.isfinite(z['course_curvature_1_m']).any()

def test_window_scheduler_never_bridges_invalid_or_reference_changes():
 from runtime_geometric_curvature_v1 import windows
 def row(i,valid=True,aux=0.):
  return dict(frame=i,current_tracking_chain=dict(valid=valid,fresh_tracker_record=True,source='tracking',reference_drive_direction='forward',projection_index=0),tracker_state_after_act=dict(active_direction='forward',pending_direction=None,braking_state='tracking'),current_reference_waypoints_after_act=[dict(x_m=0,y_m=0,yaw_rad=0,drive_direction='forward',main_command_rad_s=.5,auxiliary_angle_rad=aux),dict(x_m=1,y_m=0,yaw_rad=0,drive_direction='forward',main_command_rad_s=.5,auxiliary_angle_rad=aux)])
 r=[row(i) for i in range(35)]+[row(35,False)]+[row(i,aux=.1) for i in range(36,40)]
 x=windows(r)
 assert [(a,b) for a,b,_ in x]==[(0,30),(30,35),(36,40)]
