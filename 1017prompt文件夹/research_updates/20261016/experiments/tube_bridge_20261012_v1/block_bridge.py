"""Block-level adapter for existing runtime.execute; not live-validated."""
import math,json
import numpy as np
from bridge import Bridge
class BlockBridge(Bridge):
 def __init__(self,path,sequence_id):
  super().__init__(path,sequence_id);self.block_index=0;self.busy=False;self.contact=False
 def step(self,execute):raise ValueError('block mode only')
 def block(self,runtime):
  try:self.journal.require_owner()
  except BaseException:self.failed=True;raise
  if self.journal.closed or self.journal.poisoned or self.busy or self.failed or self.contact or self.block_index>=5 or self.collector.proposal is None:
   self.failed=True;self.journal.close();raise ValueError('block dispatch unavailable')
  self.journal.require_owner();self.busy=True
  try:
   if runtime._action_protocol.controls_per_block!=5:raise ValueError('original5-control protocol required')
   begin=float(runtime._clock_metadata()[0]);expected_begin=self.collector.proposal.time_s+self.block_index*.1
   if not math.isfinite(begin) or abs(begin-expected_begin)>1e-7:raise ValueError('block start clock mismatch')
   diag=runtime._physics_diagnostics;keys=('raw_command_target','simulation_time_s','position_world_xy')
   if not all(k in diag for k in keys) or len({len(diag[k]) for k in keys})!=1:raise ValueError('aligned physics diagnostics required')
   start=len(diag['simulation_time_s']);offset=self.block_index*5
   program=self.collector.proposal.raw_targets[offset:offset+5]
   self.journal.append(dict(type='block_dispatch_intent',index=self.block_index,program=program))
   result=runtime.execute(np.asarray(program,dtype=np.float32).reshape(10))
   if self.failed or self.journal.poisoned or self.journal.closed:raise ValueError('reentrant block attempt')
   diag=runtime._physics_diagnostics
   if any(len(diag[k])!=start+5 for k in keys):raise ValueError('exact5 physics diagnostics required')
   records=[]
   for j in range(5):
    end=float(diag['simulation_time_s'][start+j]);expected=self.collector.proposal.time_s+(offset+j+1)*.02
    if not math.isfinite(end) or abs(end-expected)>1e-7:raise ValueError('physics clock mismatch')
    pair=tuple(float(v) for v in diag['raw_command_target'][start+j]);xy=tuple(float(v) for v in diag['position_world_xy'][start+j]);records.append(dict(end_time=end,actual_raw_pair=pair,end_world_xy=xy))
   self.journal.append(dict(type='block_feedback',index=self.block_index,records=records,physical_contact=bool(result.physical_contact),collision=bool(result.collision)))
   for j,record in enumerate(records):
    self.collector.consume(self.collector.proposal.time_s+(offset+j)*.02,record['actual_raw_pair']);self.collector.supervise(record['end_time'],record['end_world_xy'])
   self.block_index+=1;self.contact=bool(result.physical_contact or result.collision)
   if self.contact or self.block_index==5:self.journal.close()
  except BaseException:self.failed=True;self.journal.close();raise
  finally:self.busy=False
 def qualify(self):
  if self.contact:return dict(scorable=False,reason='physical_contact_censored',trusted_collector_verified=False)
  return super().qualify()
