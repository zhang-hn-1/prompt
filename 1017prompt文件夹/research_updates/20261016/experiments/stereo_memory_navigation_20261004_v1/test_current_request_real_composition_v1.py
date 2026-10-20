"""Actual frozen policy composition single act, current captured state, no physics replay."""
import json
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from benchmark.policies import EpisodeContext,PolicyObservation,RGB_SHAPE
from scripts.run_pointgoal_benchmark import load_action_protocol
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from command_domain_cap import with_command_domain_cap
from fixed_reference_pacing_harness_v1 import with_frozen_reference
from current_request_context_v1 import with_current_request_context

H=Path(__file__).parent
class G:
 def set_source(self,s):pass
 def update_points(self,*args):pass
 def diagnostics(self):return {}

def test_readonly_context_actual_composition_actions_identical_single_emission():
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 cls=with_frozen_reference(with_command_domain_cap(with_cusp_coast_acquisition(with_first_primitive_direction(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(PriorRGBPolicy))))),json.loads((H/'straight_pacing_fixed_reference_v1.json').read_text()))
 state=json.loads((H/'fixed_reference_pacing_b0_run_v2/states.json').read_text())[60];history=np.load(H/'fixed_reference_pacing_b0_run_v2/history/frame_060.npz');o=PolicyObservation(stereo_left_rgb=np.zeros(RGB_SHAPE,np.uint8),stereo_right_rgb=np.zeros(RGB_SHAPE,np.uint8),action_history=history['action_history'],position_world_xy=np.asarray(state['position'][:2]),yaw_world_rad=float(np.arctan2(2*(state['quaternion'][0]*state['quaternion'][3]+state['quaternion'][1]*state['quaternion'][2]),1-2*(state['quaternion'][2]**2+state['quaternion'][3]**2))),proprio_history=history['proprio_history'],quaternion_world_wxyz_history=history['quaternion_world_wxyz_history'])
 actions=[]
 for C in [cls,with_current_request_context(cls)]:
  p=C(protocol,memory_mode='current');p.reset(EpisodeContext('diagnostic',False,None,protocol.protocol_id));p.grid=G();p.last_frame=59;p.time=5.9
  p.set_source_packet({'frame':60,'time_s':state['time_s'],'records':[]});p.set_current_packet(dict(frame=60,time_s=state['time_s'],root_xyz=np.array(state['position']),points_world=np.empty((0,3))))
  a=p.act(o,np.array([8.,0.]));actions.append(a)
  assert p.tracker.tracking_diagnostics['valid'] and p.tracker.tracking_diagnostics['source']=='tracking'
 assert np.array_equal(actions[0],actions[1])
 assert len(p.tracker._tracking_records)==1
 assert p.current_request_context.emission_count==1
 assert p.current_request_context.geometry_computed_this_call
 assert p.current_request_context.issued_request==tuple(actions[1])
 assert p.current_request_context.applied_target is None
 assert np.array_equal(actions[0][1::2],actions[1][1::2])
 # Ordinary geometry replacement preserves telemetry; next call must bind a new emission.
 old=p.tracker._tracking_records
 from depth_local_curve import install_local_reference
 install_local_reference(p.tracker,p.plan)
 p.set_source_packet({'frame':61,'time_s':state['time_s']+.1,'records':[]})
 p.set_current_packet(dict(frame=61,time_s=state['time_s']+.1,root_xyz=np.array(state['position']),points_world=np.empty((0,3))))
 p.act(o,np.array([8.,0.]))
 assert p.tracker._tracking_records is old
 assert len(p.tracker._tracking_records)==2
 assert not p.current_request_context.records_reset_during_act
 assert p.current_request_context.frame==61
 assert len(p.request_context_records)==2
