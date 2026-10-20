"""Fixed local reference diagnostic; source/planner do not choose future path."""
from pathlib import Path
import numpy as np
from prior_rgb_navigation import PriorRGBPolicy,PriorRGBGrid
from outgoing_lookahead import make_endpoint_oracle
from velocity_pursuit_consumer import with_velocity_pursuit
from oracle_rgb_shadow import reference_from_arrays
from depth_local_curve import install_local_reference
from observation_handoff import ObservationHandoff
from risk_grid import limit_action
H=Path(__file__).resolve().parent

class FixedReferencePolicy(PriorRGBPolicy):
 policy_name='fixed_local_reference_pursuit_diagnostic'
 online_input_permissions=frozenset({'cached_complete_local_reference_prior','current_known_pose','current_proprio','pointgoal','current_RGB_readiness_only'})
 def __init__(self,protocol,variant):
  if variant not in ('B0','velocity'):raise ValueError('one declared candidate')
  self.variant=variant;super().__init__(protocol,memory_mode='current')
 def reset(self,context):
  super().reset(context)
  if self.variant=='velocity':self.tracker=with_velocity_pursuit(type(self.tracker))(self.protocol,lookahead_m=.5)
  self.tracker.mapping_records=[];self.gate=ObservationHandoff()
  d=np.load(H/'fixed_pursuit_reference_v1.npz');self.plan=reference_from_arrays(d['pose'],d['direction'],d['command'],float(d['length']),0);install_local_reference(self.tracker,self.plan)
  self.local_reference_version=1;self.plan_meta=dict(unknown_fraction=1.,fixed_reference_prior=True,not_current_observation=True);self.last_plan=0.
 def act(self,observation,goal):
  if self._source is None or self._packet is None:raise RuntimeError('current source/state required')
  packet=self._packet;t=float(packet['time_s']);xy=np.asarray(observation.position_world_xy,float)
  if np.linalg.norm(packet['root_xyz'][:2]-xy)>1e-5:raise ValueError('state mismatch')
  if self.grid is None:self.grid=PriorRGBGrid(xy-20,memory_mode='current')
  self.grid.set_source(self._source);self.grid.update_points(packet['points_world'],packet['root_xyz'],t)
  speed=float(np.linalg.norm(observation.proprio_history[-1,:2]));prefix=self.gate.choose(t,speed,self._source,np.zeros(10,np.float32))
  if self.gate.handed_off:action=limit_action(self.tracker.act(observation,goal),1.)
  else:
   self.tracker._begin_tracking_audit(observation);self.tracker._braking_state='fixed_observation';action=self.tracker._request_action(float(prefix[0]),float(prefix[1]),source='fixed_observation')
  self._packet=None;self.last_frame=int(packet['frame']);self.time=t
  self.records.append(dict(frame=self.last_frame,time_s=t,local_plan_created=False,local_reference_version=1,plan=self.plan_meta,unknown_fraction=1.,certified_safe=False,request=action.tolist(),fixed_reference=True,reference_installed_time_s=0.,reference_age_s=t,reference_decision=dict(decision='fixed_reference_no_replan'),observation_handoff_time_s=self.gate.handoff_time_s,source_layer='RGB_readiness_only_knownpose_fixed_reference',source_frame=int(self._source['frame']),source_time_s=float(self._source['time_s']),source_age_s=t-float(self._source['time_s']),nominal_policy_fields_not_applied=False,**self.grid.diagnostics()))
  return action
