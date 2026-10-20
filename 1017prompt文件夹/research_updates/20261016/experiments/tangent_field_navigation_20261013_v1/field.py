import numpy as np

def target(position,start,end,lookahead):
 p=np.asarray(position,float);a=np.asarray(start,float);b=np.asarray(end,float);L=float(lookahead)
 if any(x.shape!=(2,) for x in [p,a,b]) or not np.isfinite(np.r_[p,a,b,L]).all() or L<=0:raise ValueError('finite geometry and positive existing lookahead')
 d=b-a;n=float(np.linalg.norm(d))
 if n<=1e-9:raise ValueError('nonzero current segment')
 t=d/n;normal=np.array([-t[1],t[0]]);u=float(np.clip(np.dot(p-a,t),0,n));projection=a+u*t;e=float(np.dot(p-projection,normal));direction=t-normal*e/L
 return p+L*direction/np.linalg.norm(direction)

def with_tangent_field(base):
 class TangentField(base):
  def act(self,observation,goal):
   self.field_last=None;self._field_position=np.asarray(observation.position_world_xy,float).copy()
   try:return super().act(observation,goal)
   finally:self._field_position=None
  def _lookahead_target(self,lookahead_m):
   original=super()._lookahead_target(lookahead_m);reference,xy=self._require_reference();i=self._path_index;p=self._field_position
   if p is None:raise RuntimeError('current actual act only')
   if i+1>=len(reference) or reference[i].drive_direction!=reference[i+1].drive_direction:return original
   # Do not change acquisition/turning at a cusp within existing lookahead.
   stop=i+1
   while stop+1<len(reference) and reference[stop].drive_direction==reference[i].drive_direction:stop+=1
   delta=xy[i+1]-xy[i];length=float(np.linalg.norm(delta))
   if length<=1e-9:return original
   progress=float(np.clip(np.dot(p-xy[i],delta)/length,0,length))
   arc=float(np.linalg.norm(np.diff(xy[i:stop+1],axis=0),axis=1).sum()-progress)
   if arc<float(lookahead_m):return original
   try:new=target(p,xy[i],xy[i+1],lookahead_m)
   except ValueError:return original
   self.field_last=dict(position=p.tolist(),start=xy[i].tolist(),end=xy[i+1].tolist(),lookahead=float(lookahead_m),original_target=original.tolist(),new_target=new.tolist(),path_index=int(i),drive_direction=reference[i].drive_direction,current_only=True)
   return new
 return TangentField
