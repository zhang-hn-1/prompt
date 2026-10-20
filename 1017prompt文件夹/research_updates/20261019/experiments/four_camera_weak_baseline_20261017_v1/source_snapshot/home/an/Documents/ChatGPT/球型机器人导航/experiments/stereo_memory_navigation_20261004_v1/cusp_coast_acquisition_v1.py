"""One empirical incoming-cusp acquisition call, not a certified stopping model."""
from pathlib import Path
import json,numpy as np
CONFIG=json.loads((Path(__file__).resolve().parent/'E1_cusp_coast_acquisition_design_v1.json').read_text())

def decide(*,reference,xy,current_index,position,active,armed,speed,main_joint_velocity,bank_rad):
 result=dict(acquire_index=None,valid_current_query=True,reason='ineligible',heuristic_not_certified_stopping_bound=True,full_initial_state_coverage_not_proven=True)
 p=np.asarray(position,float);q=np.asarray(xy,float)
 if p.shape!=(2,) or q.ndim!=2 or q.shape[1]!=2 or len(reference)!=len(q) or not np.isfinite(np.r_[p,q.ravel(),speed,main_joint_velocity,bank_rad]).all():raise ValueError('finite actual current state and installed reference')
 if not 0<=current_index<len(reference) or active not in ('forward','reverse'):raise ValueError('valid existing tracker index/direction')
 if not armed or active!='forward' or reference[current_index].drive_direction!=active:return result
 lo,hi=CONFIG['speed_domain_m_s'];jlo,jhi=CONFIG['main_joint_velocity_domain_rad_s']
 if not lo-1e-8<=speed<=hi+1e-8 or not jlo-1e-8<=main_joint_velocity<=jhi+1e-8 or abs(bank_rad)>CONFIG['bank_abs_max_rad']+1e-8:
  result['reason']='outside_observed_marginal_domain';return result
 cusp=next((i for i in range(current_index+1,len(reference)) if reference[i].drive_direction!=active),None)
 if cusp is None:result['reason']='no_incoming_cusp';return result
 start=current_index
 while start>0 and reference[start-1].drive_direction==active:start-=1
 incoming=q[start:cusp+1];delta=np.diff(incoming,axis=0);length=np.linalg.norm(delta,axis=1);arc=np.r_[0.,np.cumsum(length)];denom=length**2;u=np.divide(np.sum((p-incoming[:-1])*delta,axis=1),denom,out=np.zeros(len(length)),where=denom>0);u=np.clip(u,0,1);projected=incoming[:-1]+u[:,None]*delta;d=np.linalg.norm(projected-p,axis=1);i=int(np.argmin(d));remaining=float(arc[-1]-(arc[i]+u[i]*length[i]));cross=float(d[i]);result.update(cusp_index=cusp,remaining_arc_m=remaining,cross_track_m=cross,speed_m_s=float(speed),main_joint_velocity_rad_s=float(main_joint_velocity),bank_rad=float(bank_rad),acquisition_arc_m=CONFIG['cusp_acquisition_arc_m'])
 if cross<=CONFIG['cross_track_domain_m']+1e-8 and remaining<=CONFIG['cusp_acquisition_arc_m']+1e-8:
  result.update(acquire_index=cusp,reason='empirical_coast_aware_incoming_cusp')
 else:result['reason']='incoming_cusp_not_in_empirical_window'
 return result

def with_cusp_coast_tracker(base):
 class CoastAwareCusp(base):
  def act(self,observation,goal):
   self._coast_current_observation=observation
   try:return super().act(observation,goal)
   finally:self._coast_current_observation=None
  def _project_to_reference(self,position_world_xy):
   original=super()._project_to_reference(position_world_xy);observation=self._coast_current_observation
   if observation is None:raise RuntimeError('only actual current act can evaluate cusp acquisition')
   from roll_pid import bank_from_quaternion
   reference,xy=self._require_reference();before=self._path_index;row=np.asarray(observation.proprio_history[-1],float);q=np.asarray(observation.quaternion_world_wxyz_history[-1],float)
   decision=decide(reference=reference,xy=xy,current_index=before,position=position_world_xy,active=self._active_drive_direction,armed=self._coast_enabled,speed=float(np.linalg.norm(row[:2])),main_joint_velocity=float(row[12]),bank_rad=bank_from_quaternion(q));decision['original_index_after_projection']=before;self._coast_decision=decision
   if decision['acquire_index'] is not None:
    self._path_index=decision['acquire_index'];return float(np.linalg.norm(xy[self._path_index]-position_world_xy))
   return original
 return CoastAwareCusp

def with_cusp_coast_acquisition(base):
 class CuspCoastAcquisition(base):
  def reset(self,context):
   super().reset(context);self.tracker.__class__=with_cusp_coast_tracker(type(self.tracker));self.tracker._coast_enabled=False;self.tracker._coast_decision=None;self.tracker._coast_current_observation=None
  def act(self,observation,goal):
   self.tracker._coast_enabled=self.tube_viability_armed;self.tracker._coast_decision=None
   action=super().act(observation,goal);self.records[-1]['cusp_coast_acquisition']=self.tracker._coast_decision
   return action
 return CuspCoastAcquisition
