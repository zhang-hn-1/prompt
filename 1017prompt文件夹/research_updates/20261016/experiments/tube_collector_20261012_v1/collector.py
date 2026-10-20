"""Standalone recorded-data adapter. No simulation integration or trusted provenance."""
import sys,json,math
from pathlib import Path
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'execution_tube_qualification_20261012_v1'))
sys.path.insert(0,str(ROOT/'tube_provenance_20261012_v1'))
from tube_contract import Proposal,qualify_label
from provenance import snapshot_digest,check_order

class Collector:
 def __init__(self,sequence_id):
  self.sequence=sequence_id;self.events=0;self.observations=[];self.observation_events=[];self.proposal=None;self.consumed=[];self.consume_events=[];self.labels=[];self.label_events=[]
 def event(self):
  self.events+=1;return self.events
 def observe(self,time_s,*,estimated_xy,estimated_velocity,estimated_heading):
  if self.proposal is not None:raise ValueError('proposal online inputs frozen')
  values=(time_s,*estimated_xy,*estimated_velocity,estimated_heading)
  if len(estimated_xy)!=2 or len(estimated_velocity)!=2 or not all(math.isfinite(v) for v in values):raise ValueError('finite typed state required')
  if self.observations and time_s<=self.observations[-1]['time_s']:raise ValueError('increasing observation times required')
  self.observations.append(dict(time_s=time_s,estimated_xy=tuple(estimated_xy),estimated_velocity=tuple(estimated_velocity),estimated_heading=estimated_heading));self.observation_events.append(self.event())
 def commit(self,raw_targets):
  if self.proposal is not None or not self.observations:raise ValueError('one commitment after observations required')
  self.times=tuple(o['time_s'] for o in self.observations);self.snapshot=json.dumps(self.observations,sort_keys=True,allow_nan=False,separators=(',',':')).encode()
  self.proposal=Proposal(self.times[-1],raw_targets,self.sequence,snapshot_digest(self.sequence,self.times,self.snapshot));self.commit_event=self.event()
  # Initial GT position is supervision only and must be supplied separately.
 def consume(self,time_s,raw_pair):
  if self.proposal is None:raise ValueError('commit before consume')
  self.consumed.append((time_s,tuple(raw_pair)));self.consume_events.append(self.event())
 def supervise(self,time_s,world_xy):
  if self.proposal is None:raise ValueError('commit before supervision')
  self.labels.append((time_s,tuple(world_xy)));self.label_events.append(self.event())
 def qualify(self):
  if self.proposal is None:raise ValueError('missing commitment')
  if len(self.consumed)!=25 or len(self.labels)!=26:return dict(scorable=False,reason='censored_execution',trusted_collector_verified=False)
  order=check_order(sequence_id=self.sequence,snapshot_sequence=self.sequence,observation_times=self.times,anchor_time=self.proposal.time_s,snapshot_bytes=self.snapshot,snapshot_sha=self.proposal.current_observation_sha256,commit_event=self.commit_event,observation_events=self.observation_events,consume_events=self.consume_events,future_observation_events=self.label_events)
  if not order['eligible']:return dict(scorable=False,reason=order['reason'],trusted_collector_verified=False)
  if any(not math.isfinite(t) or abs(t-(self.proposal.time_s+i*.02))>1e-7 for i,(t,_) in enumerate(self.consumed)):return dict(scorable=False,reason='consumption_clock_mismatch',trusted_collector_verified=False)
  result=qualify_label(self.proposal,times=tuple(t for t,_ in self.labels),positions=tuple(x for _,x in self.labels),raw_targets=tuple(x for _,x in self.consumed),complete=True,sequence_id=self.sequence,current_observation_sha256=self.proposal.current_observation_sha256)
  return dict(result,trusted_collector_verified=False)
