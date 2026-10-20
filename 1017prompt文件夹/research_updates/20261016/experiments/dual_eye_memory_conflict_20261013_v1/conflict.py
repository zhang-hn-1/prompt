import numpy as np
from viewpoint_confirmation import ViewpointConfirmationGrid
from types import SimpleNamespace
class DualEyeConflictGrid(ViewpointConfirmationGrid):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.bounds={};self.seen_frames=set();self.conflicts={};self.demotions=[];self.last_probe_frame=-1
 def consume_with_current(self,records,now,root,views,current,poses):
  t=float(now)
  # Current source may be the latest capture, never interpreted as a new capture at intervening action ticks.
  st=float(current['time_s']);pt=float(poses['time_s']);f=int(current['frame'])
  if not np.isfinite([t,st,pt]).all() or st>t or st!=pt:raise ValueError('causal matching current source/pose')
  super().consume_records(records,t,root,views)
  for r in records:
   sf=int(r['frame'])
   if sf in self.seen_frames:continue
   self.seen_frames.add(sf);w=np.asarray(r['world'],float).reshape(-1,3);cells=self.cells(w[:,:2])
   for c in np.unique(cells,axis=0):
    if not self.inside(c[None])[0]:continue
    k=tuple(c);self.conflicts.pop(k,None);sel=(cells==c).all(1);points=w[sel];lo=points.min(0);hi=points.max(0);old_sigma=float(np.max(np.asarray(r['sigma_m'])[sel]))
    if k in self.bounds:lo=np.minimum(lo,self.bounds[k][0]);hi=np.maximum(hi,self.bounds[k][1]);old_sigma=max(old_sigma,self.bounds[k][2])
    self.bounds[k]=(lo,hi,old_sigma)
  if f==self.last_probe_frame:return []
  self.last_probe_frame=f;world=np.asarray(current['world'],float);sigma=np.asarray(current['sigma_m'],float);q=np.asarray(current['query_pixel_id'],int);p=np.asarray(current['partner_pixel_id'],int)
  if world.shape!=(len(q),3) or p.shape!=q.shape or sigma.shape!=q.shape or not np.isfinite(world).all() or not np.isfinite(sigma).all():raise ValueError('aligned finite actual endpoints')
  lookup=[]
  for eye,ids in [('left',q),('right',p)]:
   T=np.asarray(poses[eye+'_camera_usd_world'],float);rot=T[:3,:3]@np.diag([1,-1,-1]);depth=(world-T[:3,3])@rot;z=np.full(640*480,np.inf);s=np.full(640*480,np.inf)
   if len(ids):
    # Conservative minimum depth and maximum uncertainty when partner pixels duplicate.
    np.minimum.at(z,ids,depth[:,2]);s.fill(0);np.maximum.at(s,ids,sigma)
   lookup.append((T,rot,z,s,ids))
  changed=[];clock=self._confirmed_clock
  for c in np.argwhere(np.isfinite(clock)&(t-clock>10.)):
   k=tuple(c)
   if k not in self.bounds:continue
   lo,hi,old_sigma=self.bounds[k];corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]]);ok=True
   for T,rot,z,s,ids in lookup:
    opt=(corners-T[:3,3])@rot
    if np.any(opt[:,2]<=0):ok=False;break
    pix=np.floor(opt[:,:2]/opt[:,2,None]*584.8888888888889+np.array([319.5,239.5])+.5).astype(int)
    if np.any(pix<0) or np.any(pix[:,0]>=640) or np.any(pix[:,1]>=480):ok=False;break
    ix=pix[:,1]*640+pix[:,0]
    # No interpolation, pixel neighbourhood expansion, absence/nohit, or compatibility clearing.
    if not np.all(np.isin(ix,ids)) or not np.all(np.isfinite(z[ix])):ok=False;break
    margin=.3+2*(old_sigma+s[ix])+float(current.get('pose_sigma_m',0))
    if not np.all(z[ix]-opt[:,2]>margin):ok=False;break
   if not ok:continue
   epochs=self.conflicts.setdefault(k,set());diverse=any(np.linalg.norm(views[f]['position_world']-views[e]['position_world'])>=self.resolution-1e-8 for e in epochs);epochs.add(f)
   if not diverse:continue
   self._confirmed_clock[k]=-np.inf;self._persistent_clock[k]=-np.inf;self.hazard_acquisition_time[k]=-np.inf;self.last_seen[k]=-np.inf;self.labels[k]=-1;self.codes[k]=0;self._confirmation_epochs.pop(k,None);self.bounds.pop(k,None);self.conflicts.pop(k,None);changed.append(k);self.demotions.append(dict(cell=list(k),time_s=t,frame=f,unknown_not_free=True))
  self.memory=SimpleNamespace(floor_time=np.full(self.labels.shape,-np.inf),hazard_time=self.hazard_acquisition_time.copy());return changed
