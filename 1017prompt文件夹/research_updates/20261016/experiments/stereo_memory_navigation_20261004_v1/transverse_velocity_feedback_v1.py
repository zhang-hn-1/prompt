"""One empirical actual-transverse-velocity auxiliary consumer; not a dynamics model."""
import math
import numpy as np
from benchmark.action_layers import normalize_action_request
GAIN_RAD_PER_M_S=1.0

def with_transverse_velocity_feedback(base,*,enabled):
 class TransverseVelocityFeedback(base):
  def act(self,observation,goal):
   action=np.asarray(super().act(observation,goal),np.float32).copy();parent=action.copy();row=self.records[-1];c=row.get('current_tracking_chain',{});tr=self.tracker
   eligible=bool(c.get('fresh_tracker_record',False) and c.get('valid',False) and c.get('source')=='tracking' and c.get('reference_drive_direction')=='forward' and tr._active_drive_direction=='forward' and tr._pending_drive_direction is None and tr._braking_state=='tracking' and np.all(parent[::2]==parent[0]) and parent[0]>0 and np.all(parent[1::2]==parent[1]))
   velocity=None;correction=None;sign_coherent=None
   if eligible:
    theta=float(c['reference_motion_tangent_rad']);body=float(c['root_heading_rad']);vx=float(c['velocity_world_x_m_s']);vy=float(c['velocity_world_y_m_s'])
    if not np.isfinite([theta,body,vx,vy]).all():raise ValueError('finite current reference and measured XY velocity')
    sign_coherent=math.cos(body-theta)>0
    eligible=eligible and sign_coherent
    if eligible:velocity=-math.sin(theta)*vx+math.cos(theta)*vy;correction=GAIN_RAD_PER_M_S*velocity
    if enabled and eligible:action[1::2]=normalize_action_request(self.protocol,main_velocity=float(parent[0]),auxiliary_angle=float(parent[1])+correction,source='tracking').protocol_block[1::2]
   np.testing.assert_array_equal(action[::2],parent[::2])
   row.update(transverse_feedback_enabled=bool(enabled),transverse_feedback_eligible=eligible,transverse_feedback_parent_request=parent.tolist(),transverse_feedback_same_hemisphere_sign_domain=sign_coherent,transverse_feedback_velocity_m_s=velocity,transverse_feedback_correction_rad=correction,transverse_feedback_gain_rad_per_m_s=GAIN_RAD_PER_M_S,transverse_feedback_override=not np.array_equal(action,parent),transverse_feedback_final_issued_request=action.tolist(),current_tracking_chain_layer='current_parent_before_transverse_aux_consumption',request=action.tolist())
   return action
 return TransverseVelocityFeedback

def policy_class(enabled):
 from current_tracking_chain_telemetry_v1 import policy_class as parent
 return with_transverse_velocity_feedback(parent(True),enabled=enabled)
