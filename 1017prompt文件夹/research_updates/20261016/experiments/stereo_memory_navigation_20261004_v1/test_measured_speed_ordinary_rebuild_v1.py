"""CPU ordinary composition with explicit synthetic current positions, not physics evidence."""
import dataclasses,json
from pathlib import Path
import numpy as np
from benchmark.policies import EpisodeContext,PolicyObservation,RGB_SHAPE
from scripts.run_pointgoal_benchmark import load_action_protocol
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from command_domain_cap import with_command_domain_cap
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from measured_speed_consumer_v1 import with_measured_speed
H=Path(__file__).parent

def test_ordinary_current_speed_terminal_reserve_then_rebuild_single_emission():
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 cls=with_command_domain_cap(with_cusp_coast_acquisition(with_first_primitive_direction(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(PriorRGBPolicy)))))
 state=json.loads((H/'fixed_reference_pacing_b0_run_v2/states.json').read_text())[60];history=np.load(H/'fixed_reference_pacing_b0_run_v2/history/frame_060.npz');q=state['quaternion'];yaw=float(np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2)))
 o=PolicyObservation(stereo_left_rgb=np.zeros(RGB_SHAPE,np.uint8),stereo_right_rgb=np.zeros(RGB_SHAPE,np.uint8),action_history=history['action_history'],position_world_xy=np.array(state['position'][:2]),yaw_world_rad=yaw,proprio_history=history['proprio_history'],quaternion_world_wxyz_history=history['quaternion_world_wxyz_history'])
 # Explicit ideal CPU geometry, not claim that the historical pose qualified.
 yaw=0.;o=dataclasses.replace(o,position_world_xy=np.array([0.,0.]),yaw_world_rad=0.,quaternion_world_wxyz_history=np.tile(np.array([1.,0.,0.,0.]),(len(history['quaternion_world_wxyz_history']),1)))
 rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);worldgoal=o.position_world_xy+rotation@np.array([8.,0.]);actions=[]
 for C in [cls,with_measured_speed(cls)]:
  p=C(protocol,memory_mode='current');p.reset(EpisodeContext('cpu_synthetic_current',False,None,protocol.protocol_id));p.grid=RepeatedExposureMemoryGrid(o.position_world_xy-20,10,static_scene_assumption=True);p.last_frame=59;p.time=5.9
  def act(frame,t,xy):
   current=dataclasses.replace(o,position_world_xy=np.array(xy));root=np.r_[xy,state['position'][2]]
   p.set_source_packet(dict(frame=frame,time_s=t,records=[]));p.set_current_packet(dict(frame=frame,time_s=t,root_xyz=root,points_world=np.empty((0,3))))
   return p.act(current,rotation.T@(worldgoal-xy))
  a=act(60,6.,o.position_world_xy);path=p.tracker._reference_xy.copy();direction=(path[-1]-path[-2])/np.linalg.norm(path[-1]-path[-2]);oldlist=p.tracker._tracking_records
  b=act(61,6.1,path[-1]-.6*direction)
  assert not p.records[-1]['local_plan_created']
  if C is not cls:
   assert not p.records[-1]['measured_speed_control_applied']
   assert 'preserve_original_within_same_reserve' in p.records[-1]['measured_speed_control_reasons']
  for f in range(62,65):act(f,6.+.1*(f-60),path[-1]-.6*direction)
  c=act(65,6.5,path[-1]-.3*direction)
  assert p.records[-1]['local_plan_created'] and p.local_reference_version==2
  assert p.tracker._tracking_records is oldlist and len(oldlist)==6
  actions.append((a,b,c))
 assert np.all(actions[0][0][::2]==.5) and np.all(actions[1][0][::2]==1.)
 assert np.array_equal(actions[0][1],actions[1][1])
 for a,b in zip(*actions):assert np.array_equal(a[1::2],b[1::2])
