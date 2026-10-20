"""One fixed episodic outcome retriever, not a source observation or runtime controller."""
from dataclasses import dataclass
import math
import numpy as np
HORIZON=5

def owned(value,shape):
 a=np.array(value,dtype=np.float64,copy=True)
 if a.shape!=shape or not np.isfinite(a).all():raise ValueError('finite array with fixed shape required')
 a.setflags(write=False);return a

@dataclass(frozen=True)
class Proposal:
 commands:np.ndarray
 created_at:float
 producer:str

def mint_hold_proposal(main,aux,now):
 values=np.array([main,aux,now],float)
 if not np.isfinite(values).all() or now<0 or abs(main)>2 or abs(aux)>math.pi/6+1e-6:raise ValueError('finite current bounded proposal')
 return Proposal(owned(np.tile([main,aux],(5,1)),(5,2)),float(now),'hold_current_request_v1')

def proposal_from_record(record):
 if set(record)!= {'commands','created_at','producer'} or record['producer']!='hold_current_request_v1':raise ValueError('genuine hold producer required; realized logs disallowed')
 a=owned(record['commands'],(5,2));p=mint_hold_proposal(a[0,0],a[0,1],record['created_at'])
 if not np.array_equal(a,p.commands):raise ValueError('hold proposal must be constant')
 return p

def features(packet,now):
 if set(packet)!={'times','proprio','past_actions','yaw_current','privilege'} or packet['privilege']!='simulator_state_diagnostic':raise ValueError('explicit known state diagnostic privilege required')
 t=np.asarray(packet['times'],float)
 if t.shape!=(5,) or not np.isfinite(t).all() or not np.isfinite(now) or abs(t[-1]-now)>1e-6 or not np.allclose(np.diff(t),.1,atol=1e-6,rtol=0) or (t>now+1e-8).any():raise ValueError('current and past clocks required before payload')
 p=owned(packet['proprio'],(5,17)).copy();past=owned(packet['past_actions'],(5,2));yaw=float(packet['yaw_current'])
 if not math.isfinite(yaw):raise ValueError('finite current gauge required')
 c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,s],[-s,c]])
 p[:,:2]=p[:,:2]@rotation.T;p[:,3:5]=p[:,3:5]@rotation.T
 scales=np.array([.3]*3+[1.]*6)
 v=np.concatenate([p[:,:9]/scales,np.sin(p[:,9:12]),np.cos(p[:,9:12]),p[:,12:15],p[:,15:17]/[.5,math.pi/6]],axis=1)
 result=np.r_[v.flatten(),(past/[.5,math.pi/6]).flatten()];return owned(result,(150,))

class Memory:
 def __init__(self,entries,*,allowed_episodes,forbidden_episodes):
  allowed=frozenset(allowed_episodes);forbidden=frozenset(forbidden_episodes)
  if allowed&forbidden:raise ValueError('training/heldout episodes disjoint')
  rows=[]
  for e in entries:
   if set(e)!= {'episode','frame','feature','program','target'} or e['episode'] not in allowed or e['episode'] in forbidden:raise ValueError('library must contain only declared training episodes')
   if int(e['frame'])!=e['frame'] or e['frame']<5:raise ValueError('un-padded training anchor required')
   rows.append((str(e['episode']),int(e['frame']),owned(e['feature'],(150,)),owned(e['program'],(5,2)),owned(e['target'],(5,3))))
  self._rows=tuple(rows)
 def query(self,feature,proposal,*,now=None):
  if not isinstance(proposal,Proposal):raise ValueError('minted proposal object required')
  p=proposal_from_record(dict(commands=proposal.commands,created_at=proposal.created_at,producer=proposal.producer))
  if now is not None and (not math.isfinite(now) or abs(p.created_at-now)>1e-6):raise ValueError('proposal must exist at current decision clock')
  f=owned(feature,(150,));eligible=[]
  for i,r in enumerate(self._rows):
   d=np.abs(r[3]-p.commands)
   if (d[:,0]>.01+1e-12).any() or (d[:,1]>math.radians(2)+1e-12).any():continue
   rms=float(np.sqrt(np.mean((r[2]-f)**2)))
   if rms<=.25:eligible.append((rms,i))
  if not eligible:return dict(status='unknown',reason='no_supported_state_and_program',trajectory=None)
  dist,i=min(eligible);r=self._rows[i]
  return dict(status='supported',trajectory=r[4].copy(),library_episode=r[0],library_frame=r[1],normalized_state_rms=dist,prediction_not_observation=True,full_3d_certified=False)
