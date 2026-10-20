"""Causal feature memory for one reliability head; no GT/scene IDs/features."""
import numpy as np
from scipy.ndimage import distance_transform_edt

class CellMemory:
 def __init__(self,origin,shape,resolution):
  self.origin=np.asarray(origin,float);self.shape=tuple(shape);self.resolution=float(resolution);self.time=-np.inf;self.records={};self.max_age=35.
  if self.origin.shape!=(2,) or not np.isfinite(self.origin).all() or len(self.shape)!=2 or min(self.shape)<=0 or not np.isfinite(self.resolution) or self.resolution<=0:raise ValueError('valid geometry required')
 def ingest(self,packet,context,current_time_s):
  t=float(current_time_s);qt=float(packet['time_s'])
  if not np.isfinite(t) or not np.isfinite(qt) or qt>t or t<self.time:raise ValueError('current/past observation required before payload read')
  if float(context['time_s'])!=qt:raise ValueError('context exposure mismatch')
  allowed={'world','sigma_m','query_pixel_id','partner_pixel_id','frame','time_s','frame_width','pose_sigma_m','source_model_admitted','certified_free'}
  required={'world','sigma_m','query_pixel_id','partner_pixel_id','frame','time_s','frame_width'}
  if not required.issubset(packet) or not set(packet).issubset(allowed):raise ValueError('source whitelist')
  if set(context)!={'time_s','left_rgb','right_rgb','left_mask','left_camera_position','root_position','previous_request'}:raise ValueError('context whitelist')
  xyz=np.asarray(packet['world'],float);sigma=np.asarray(packet['sigma_m'],float)+float(packet.get('pose_sigma_m',0.));q=np.asarray(packet['query_pixel_id']);s=np.asarray(packet['partner_pixel_id']);n=len(xyz)
  l=np.asarray(context['left_rgb']);r=np.asarray(context['right_rgb']);mask=np.asarray(context['left_mask']);cam=np.asarray(context['left_camera_position'],float);root=np.asarray(context['root_position'],float);request=np.asarray(context['previous_request'],float)
  if l.ndim!=3 or l.shape[-1]!=3 or l.shape!=r.shape or l.dtype!=np.uint8 or r.dtype!=np.uint8 or mask.dtype!=bool or mask.shape!=l.shape[:2] or int(packet['frame_width'])!=l.shape[1]:raise ValueError('RGB/mask geometry')
  if xyz.shape!=(n,3) or sigma.shape!=(n,) or q.shape!=(n,) or s.shape!=(n,) or not np.isfinite(xyz).all() or not np.isfinite(sigma).all() or (sigma<0).any():raise ValueError('point fields')
  if not np.issubdtype(q.dtype,np.integer) or not np.issubdtype(s.dtype,np.integer) or (q<0).any() or (s<0).any() or (q>=l.shape[0]*l.shape[1]).any() or (s>=l.shape[0]*l.shape[1]).any():raise ValueError('pixel IDs')
  if cam.shape!=(3,) or root.shape!=(3,) or request.shape!=(10,) or not all(np.isfinite(a).all() for a in [cam,root,request]):raise ValueError('current sensor context')
  gray=l.astype(float)@np.array([.299,.587,.114]);gy,gx=np.gradient(gray);grad=np.hypot(gx,gy);distance=distance_transform_edt(~mask);flat=gray.ravel();gflat=grad.ravel();dflat=distance.ravel();residual=np.mean(np.abs(l.reshape(-1,3)[q].astype(float)-r.reshape(-1,3)[s].astype(float)),axis=1);staged={};frame=int(packet['frame'])
  for j in range(n):
   if not (.05<xyz[j,2]<=1.5):continue
   cell=tuple(np.rint((xyz[j,:2]-self.origin)/self.resolution).astype(int))
   if min(cell)<0 or cell[0]>=self.shape[0] or cell[1]>=self.shape[1]:continue
   key=(frame,'left',int(q[j]),qt);record=dict(cell=cell,world=xyz[j].tolist(),sigma=float(sigma[j]),query=key,source=(frame,'right',int(s[j]),qt),camera=cam.tolist(),gray=float(flat[q[j]]),gradient=float(gflat[q[j]]),RGB_residual=float(residual[j]),mask_distance=float(dflat[q[j]]),body_range=float(np.linalg.norm(xyz[j]-root)),previous_main=float(request[::2].mean()),previous_aux=float(request[1::2].mean()))
   prior=staged.get(key,self.records.get(key))
   if prior is not None and prior!=record:raise ValueError('same original query identity changed')
   if t-qt<=self.max_age:staged[key]=record
  self.records={k:v for k,v in self.records.items() if t-v['query'][3]<=self.max_age};self.records.update(staged);self.time=t
 def snapshot(self,current_time_s):
  t=float(current_time_s)
  if not np.isfinite(t) or t<self.time:raise ValueError('no future state in past snapshot')
  cells={}
  for r in self.records.values():
   if t-r['query'][3]<=self.max_age:cells.setdefault(r['cell'],[]).append(r)
  keys=sorted(cells);features=[];last=[];query_ids=[];source_ids=[]
  for key in keys:
   records=cells[key];xyz=np.array([r['world'] for r in records]);sigma=np.array([r['sigma'] for r in records]);times=np.array([r['query'][3] for r in records]);exposures={r['query'][0]:r['camera'] for r in records};cam=np.array(list(exposures.values()));span=np.linalg.norm(cam-cam[0],axis=1).mean();baseline=np.linalg.norm(cam[:,None]-cam[None,:],axis=-1).max();gray=np.array([r['gray'] for r in records]);body_range=np.array([r['body_range'] for r in records]);f=[np.log1p(len(exposures)),np.log1p(len(records)),(t-times.max())/35,(t-times.min())/35,xyz[:,2].mean()/1.5,xyz[:,2].std()/.1,sigma.mean()/.1,sigma.max()/.1,xyz[:,0].std()/.1,xyz[:,1].std()/.1,span/3,gray.mean()/255,gray.std()/255,np.mean([r['gradient'] for r in records])/255,np.mean([r['RGB_residual'] for r in records])/255,baseline/3,np.mean([r['previous_main'] for r in records])/1.5,np.mean([r['previous_aux'] for r in records])/(np.pi/2),np.mean([r['mask_distance'] for r in records])/640,body_range.mean()/3,np.mean(sigma/np.maximum(body_range,1e-9))];assert len(f)==21 and np.isfinite(f).all();features.append(f);last.append(times.max());query_ids.append([r['query'] for r in records]);source_ids.append([r['source'] for r in records])
  return dict(time_s=t,cells=np.array(keys,int).reshape(-1,2),features=np.array(features,float).reshape(-1,21),last_query_times=np.array(last),query_ids=query_ids,source_ids=source_ids,model_outputs_are_observed=False,model_supported_static_prior_only=True,source_model_admitted=False,not_a_depth_correctness_certificate=True)
