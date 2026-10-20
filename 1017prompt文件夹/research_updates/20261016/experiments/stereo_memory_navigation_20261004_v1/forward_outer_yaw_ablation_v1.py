"""One final forward-aux request consumption ablation, parent chain retained."""
import numpy as np
from benchmark.action_layers import normalize_action_request

def with_forward_outer_yaw_ablation(base,*,enabled):
 class ForwardOuterYawAblation(base):
  def act(self,o,goal):
   action=np.asarray(super().act(o,goal),np.float32).copy();before=action.copy();row=self.records[-1];chain=row.get('current_tracking_chain',{});tr=self.tracker
   eligible=bool(chain.get('fresh_tracker_record',False) and chain.get('valid',False) and chain.get('source')=='tracking' and tr._active_drive_direction=='forward' and tr._pending_drive_direction is None and tr._braking_state=='tracking' and np.all(action[::2]==action[0]) and action[0]>0 and np.all(action[1::2]==action[1]))
   encoded=None
   if eligible:
    mapping=chain['calibrated_bank_mapping'];assert mapping['active'] and mapping['direction']=='forward';encoded=float(mapping['encoded_aux_rad']);assert np.isfinite(encoded)
    if enabled:
     normalized=normalize_action_request(self.protocol,main_velocity=float(action[0]),auxiliary_angle=encoded,source='tracking');action[1::2]=normalized.protocol_block[1::2]
   np.testing.assert_array_equal(action[::2],before[::2])
   row.update(forward_outer_yaw_enabled=bool(enabled),forward_outer_yaw_eligible=eligible,forward_outer_yaw_override=not np.array_equal(action,before),forward_outer_yaw_parent_request=before.tolist(),forward_outer_yaw_current_encoded_aux_rad=encoded,forward_outer_yaw_chain_layer='current_parent_tracker_request_before_outer_aux_consumption',forward_outer_yaw_final_issued_request=action.tolist(),request=action.tolist())
   return action
 return ForwardOuterYawAblation

def policy_class(enabled):
 from current_tracking_chain_telemetry_v1 import policy_class as parent
 return with_forward_outer_yaw_ablation(parent(True),enabled=enabled)
