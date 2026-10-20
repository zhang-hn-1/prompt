"""Observation-only current/past RGB witness around frozen, rejected ORB.

No policy/reference/action/depth/occupancy may enter this interface. Known
camera poses are privileged diagnostic inputs, not estimated localization.
"""
from pathlib import Path
import sys,json,copy,time
import numpy as np
from feature_tracks import extract
from active_observation import guarded_predict
from point_support_shadow import PointSupportMap,finite_scalar
H=Path(__file__).resolve().parent
FIELDS={'frame','time_s','left_rgb','right_rgb','left_camera_usd_world','right_camera_usd_world'}

def sanitize(packet):
 if set(packet)!=FIELDS:raise ValueError('only RGB/current known camera poses and clock allowed')
 t=finite_scalar(packet['time_s']);i=int(packet['frame'])
 if i!=packet['frame'] or i<0 or t<0:raise ValueError('invalid frame/clock')
 result=dict(frame=i,time_s=t)
 for eye in ['left','right']:
  rgb=np.asarray(packet[eye+'_rgb']);pose=np.asarray(packet[eye+'_camera_usd_world'],float)
  if rgb.ndim!=3 or rgb.shape[-1]!=3 or rgb.dtype!=np.uint8 or pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError('invalid image/pose')
  result[eye+'_rgb']=rgb.copy();result[eye+'_camera_usd_world']=pose.copy()
 return result

class OnlineRGBShadow:
 def __init__(self,masks):
  self.masks={k:np.asarray(v,bool).copy() for k,v in masks.items()};self.history=[];self.last_frame=-1;self.time=-np.inf
  self.K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
  self.map=PointSupportMap([-20.,-20.],(400,400),.1);self.latest=None
 def observe(self,packet):
  packet=sanitize(packet);t=packet['time_s'];i=packet['frame']
  if t<=self.time or i!=self.last_frame+1:raise ValueError('strict consecutive current capture required')
  begin=time.perf_counter();pair=[];features={}
  for eye in ['left','right']:
   rgb=packet[eye+'_rgb'];mask=self.masks[eye]
   if mask.shape!=rgb.shape[:2]:raise ValueError('mask shape mismatch')
   xy,desc,size=extract(rgb,mask);pose=packet[eye+'_camera_usd_world']
   pair.append(dict(frame=i,time_s=t,eye=eye,xy=xy,desc=desc,size=size,pose=pose))
   for k,v in [('xy',xy),('desc',desc),('size',size),('pose',pose)]:features[eye+'_'+k]=v
  current=guarded_predict(pair[0],[pair[1]],self.K);causal=guarded_predict(pair[0],[pair[1]]+self.history,self.K)
  self.map.update(causal['points'],t,0.);snapshot=self.map.snapshot(t)
  self.history=(self.history+pair)[-6:];self.time=t;self.last_frame=i
  result=dict(frame=i,time_s=t,current=current,causal=causal,source_views=causal['source_views'],records=snapshot['records'],floor_cells=int((snapshot['labels']==0).sum()),hazard_cells=int((snapshot['labels']>0).sum()),certified_free=False,source_model_admitted=False,action_authorized=False,pose_sigma_m=0.,seconds=time.perf_counter()-begin)
  self.latest=dict(result=copy.deepcopy(result),features=features,map=snapshot)
  return copy.deepcopy(result)

def replay_shadow(packets,time_s,*,masks):
 t=finite_scalar(time_s);shadow=OnlineRGBShadow(masks);last=None
 for p in packets:
  pt=finite_scalar(p['time_s'])
  if pt>t:continue
  last=shadow.observe(p)
 return last

def reference_from_arrays(poses,directions,commands,length,expanded):
 from benchmark.kinodynamic_planning import KinodynamicWaypoint,KinodynamicPlanResult
 poses=np.asarray(poses,float);commands=np.asarray(commands,float);directions=np.asarray(directions)
 if poses.shape!=(len(directions),3) or commands.shape!=(len(directions),2) or len(directions)<2 or not np.isfinite(poses).all() or not np.isfinite(commands).all() or not np.isin(directions,[-1,1]).all():raise ValueError('invalid cached reference')
 waypoints=tuple(KinodynamicWaypoint(*pose,'forward' if d>0 else 'reverse',*c) for pose,d,c in zip(poses,directions,commands))
 return KinodynamicPlanResult('success',waypoints,int(expanded),finite_scalar(length),int((directions[1:]!=directions[:-1]).sum()))

def make_oracle(protocol):
 from benchmark.policies import OraclePathPolicy
 # Same frozen composition; no script imports that install globals or run a simulation.
 old=H.parent/'oracle_feasible_route_20260929_v9/hybrid'
 sys.path.insert(0,str(old))
 from bank_mapping import BankMap,make_policy
 from gated_speed import with_confident_speed
 cls=with_confident_speed(make_policy(OraclePathPolicy,BankMap(json.loads((old/'bank_mapping.json').read_text()))))
 return cls(protocol,lookahead_m=.5)
