"""Independent B0 observer with the same frozen measured initial prefix, no physics API."""
import numpy as np
from front_view_acquisition_v1 import FrontViewAcquisition
from guarded_reference import boundary_state

def with_front_view_baseline_shadow(base):
 class FrontViewBaselineShadow(base):
  def reset(self,context):
   super().reset(context);self.front_view=FrontViewAcquisition()
  def act(self,observation,goal):
   if self._packet is None or self._source is None:raise RuntimeError('current shadow packets required')
   packet=self._packet;t=packet['time_s'];xy=np.asarray(observation.position_world_xy,float)
   if np.linalg.norm(packet['root_xyz'][:2]-xy)>1e-5:raise ValueError('current shadow pose mismatch')
   req=self.front_view.choose(t,xy,float(np.linalg.norm(observation.proprio_history[-1,:2])),self._source)
   if self.front_view.failed:raise RuntimeError('B0 initial prefix failed before physical action')
   if req is None:return super().act(observation,goal)
   before=boundary_state(self.tracker);plan=self.plan;last_plan=self.last_plan
   self.grid.set_source(self._source);self.grid.update_points(np.empty((0,3)),packet['root_xyz'],t)
   self.last_frame=packet['frame'];self.time=t;self._packet=None
   self.records.append(dict(frame=self.last_frame,time_s=t,request=req.tolist(),**self.grid.diagnostics()))
   assert boundary_state(self.tracker)==before and self.plan is plan and self.last_plan==last_plan
   return req
 return FrontViewBaselineShadow
