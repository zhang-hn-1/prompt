"""One exact target consumer; reference/index/latch/control parameters unchanged."""
import numpy as np

def exact_direction_target(xy,directions,index,position,lookahead):
 p=np.asarray(xy,float);q=np.asarray(position,float);L=float(lookahead)
 if p.ndim!=2 or p.shape[1]!=2 or len(p)<2 or len(directions)!=len(p) or q.shape!=(2,) or not np.isfinite(p).all() or not np.isfinite(q).all() or not np.isfinite(L) or L<=0 or isinstance(index,bool) or int(index)!=index or not 0<=index<len(p) or any(d not in ('forward','reverse') for d in directions):raise ValueError('finite current direction geometry')
 length=np.linalg.norm(np.diff(p,axis=0),axis=1)
 if np.any(length<=0):raise ValueError('duplicate reference geometry')
 direction=directions[index];start=index-1 if index>0 and directions[index-1]==direction else index;end=index
 while end+1<len(p) and directions[end]==direction:end+=1
 if start==end:return p[end].copy(),dict(projected_arc_m=0.,actual_arc_lookahead_m=0.,same_direction_end_index=end,target_segment_index=end)
 sub=p[start:end+1];delta=np.diff(sub,axis=0);length=np.linalg.norm(delta,axis=1);arc=np.r_[0.,np.cumsum(length)];u=np.clip(((q-sub[:-1])*delta).sum(1)/length**2,0,1);points=sub[:-1]+u[:,None]*delta;j=int(np.linalg.norm(points-q,axis=1).argmin());s=float(arc[j]+u[j]*length[j]);target_s=min(float(arc[-1]),s+L);k=min(len(length)-1,max(0,int(np.searchsorted(arc,target_s,side='right')-1)));target=sub[k]+delta[k]*((target_s-arc[k])/length[k]);global_arc=float(np.sum(np.linalg.norm(np.diff(p[:start+1],axis=0),axis=1)))
 return target,dict(projected_arc_m=global_arc+s,actual_arc_lookahead_m=target_s-s,same_direction_end_index=end,target_segment_index=start+k)

def with_exact_outgoing_lookahead(base):
 class ExactOutgoingLookahead(base):
  def reset(self,context):
   super().reset(context);self._exact_lookahead_seen_reverse=False;self._exact_lookahead_activated=False
  def _lookahead_target(self,lookahead_m):
   if self._active_drive_direction=='reverse':self._exact_lookahead_seen_reverse=True
   if not self._exact_lookahead_seen_reverse or self._active_drive_direction!='forward':return super()._lookahead_target(lookahead_m)
   reference,xy=self._require_reference()
   if reference[self._path_index].drive_direction!=self._active_drive_direction:return super()._lookahead_target(lookahead_m)
   pos=[self._tracking_audit['position_world_x_m'],self._tracking_audit['position_world_y_m']];target,diag=exact_direction_target(xy,[w.drive_direction for w in reference],self._path_index,pos,lookahead_m);self._exact_lookahead_activated=True
   self._tracking_audit.update(exact_outgoing_lookahead_active=True,exact_outgoing_lookahead_event='normal_forward_after_actual_reverse_latch',lookahead_index=diag['target_segment_index'],lookahead_accumulated_m=diag['actual_arc_lookahead_m'],lookahead_requested_m=float(lookahead_m),exact_outgoing_geometry=diag)
   return target
 return ExactOutgoingLookahead
