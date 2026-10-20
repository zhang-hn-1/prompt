"""Separately labelled RGB-knownpose experiment with explicit task assumptions."""
from types import SimpleNamespace
import numpy as np
from outgoing_lookahead import EndpointLookaheadPolicy
from risk_grid import ExplorationGrid,limit_action
from point_support_shadow import PointSupportMap,swept_indices
from dense_source_points import records

def short_goal(start,goal,distance_m=2.):
 start=np.asarray(start,float);goal=np.asarray(goal,float)
 if start.shape!=(3,) or goal.shape!=(3,) or not np.isfinite(start).all() or not np.isfinite(goal).all() or not np.isfinite(distance_m) or distance_m<=0:raise ValueError('finite task positions')
 delta=goal[:2]-start[:2];length=np.linalg.norm(delta)
 if length==0:raise ValueError('distinct task goal required')
 result=goal.copy();result[:2]=start[:2]+delta*min(distance_m,length)/length
 return result

def require_boolean_confidence(value):
 a=np.asarray(value)
 if a.dtype!=np.bool_:raise ValueError('only finite boolean confidence permitted')
 return a

class PriorRGBGrid(ExplorationGrid):
 def __init__(self,origin,shape=(400,400),resolution=.1,memory_mode='history'):
  if memory_mode not in ('current','history'):raise ValueError('memory mode')
  super().__init__(origin,shape=shape,resolution=resolution);self.memory_mode=memory_mode;self.mapper=PointSupportMap(origin,shape,resolution);self.source=None;self.task_ground_prior=np.ones(shape,bool);self.safe_start_prior=np.zeros(shape,bool);self._started=False;self.memory=SimpleNamespace(floor_time=np.full(shape,-np.inf),hazard_time=np.full(shape,-np.inf));self.codes=np.zeros(shape,np.uint8)
 def set_source(self,packet):self.source=packet
 def update_points(self,points,root,time_s,ground_meshes=()):
  t=float(time_s)
  if not np.isfinite(t) or t<self.time or ground_meshes:raise ValueError('noncausal query or disallowed mesh')
  if self.source is None or not np.isfinite(float(self.source['time_s'])) or float(self.source['time_s'])>t:raise ValueError('current/past RGB source required')
  if self.memory_mode=='current':self.mapper=PointSupportMap(self.origin,self.labels.shape,self.resolution)
  self.mapper.update(records(self.source),t,0.)
  if not self._started:
   c=swept_indices([np.asarray(root)[:2]],self.origin,self.resolution,.7);c=c[self.inside(c)];self.safe_start_prior[c[:,0],c[:,1]]=True;self._started=True
  s=self.mapper.snapshot(t);self.labels=s['labels'];self.codes=s['provenance'];self.memory=SimpleNamespace(floor_time=s['floor_time'],hazard_time=s['hazard_time']);self.last_seen=np.maximum(s['floor_time'],s['hazard_time']);self.time=t;self.current_disk=swept_indices([np.asarray(root)[:2]],self.origin,self.resolution,.7)
 def provenance(self):return self.codes.copy()
 def diagnostics(self):
  c=self.codes
  return dict(memory_mode=self.memory_mode,current_floor=int((c==1).sum()),historical_floor=int((c==2).sum()),current_occupied=int((c==3).sum()),historical_occupied=int((c==4).sum()),unknown=int((c==0).sum()),footprint_unknown=int(sum(self.labels[i,j]<0 for i,j in self.current_disk if 0<=i<self.labels.shape[0] and 0<=j<self.labels.shape[1])),body_prior_observed_free=0,pose_sigma_assumption_m=0.,full_3d_certified=False,task_ground_prior_is_observation=False,safe_start_prior_is_observation=False)

class PriorRGBPolicy(EndpointLookaheadPolicy):
 policy_name='experimental_rgb_knownpose_flat_ground_prior_localgoal'
 online_input_permissions=frozenset({'current_and_past_RGB_depth_estimates','current_known_pose','current_proprio','pointgoal','explicit_task_prior'})
 def __init__(self,protocol,memory_mode):self.memory_mode=memory_mode;self._source=None;super().__init__(protocol)
 def reset(self,context):self._source=None;super().reset(context)
 def set_source_packet(self,packet):self._source=packet
 def act(self,observation,goal):
  if self._source is None or self._packet is None:raise RuntimeError('current source and policy packet required')
  if self.grid is None:self.grid=PriorRGBGrid(np.asarray(observation.position_world_xy)-20,memory_mode=self.memory_mode)
  self.grid.set_source(self._source);action=super().act(observation,goal);uncapped=action.copy();action=limit_action(action,self.records[-1]['unknown_fraction'])
  self.records[-1].update(source_layer='RGB_depth_knownpose',depth_pose_privileged=False,pose_privileged=True,source_model_admitted=False,task='separate_flat_ground_prior_and_unknown_exploration',uncapped_request=uncapped.tolist(),request=action.tolist(),source_frame=int(self._source['frame']),source_time_s=float(self._source['time_s']),source_age_s=self.time-float(self._source['time_s']))
  return action
