"""Dependency-injected bridge, not a verified Isaac sensor/physics adapter."""
import sys,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for directory in ('tube_collector_20261012_v1','tube_journal_20261012_v1'):sys.path.insert(0,str(ROOT/directory))
from collector import Collector
from journal import Journal
class Bridge:
 def __init__(self,path,sequence_id):
  self.collector=Collector(sequence_id);self.journal=Journal(path);self.failed=False;self.anchored=False
 def anchor(self,time_s,*,left_rgb,right_rgb,estimated_xy,estimated_velocity,estimated_heading,source,initial_gauge,flat_ground_prior):
  if self.anchored or source!='native_imu_rolling' or type(left_rgb) is not bytes or type(right_rgb) is not bytes or not left_rgb or not right_rgb:raise ValueError('one typed native/RGB anchor required')
  if type(initial_gauge) is not bool or type(flat_ground_prior) is not bool:raise ValueError('explicit privilege flags')
  self.collector.observe(time_s,estimated_xy=estimated_xy,estimated_velocity=estimated_velocity,estimated_heading=estimated_heading)
  self.rgb=(left_rgb,right_rgb);self.source=dict(source=source,initial_gauge=initial_gauge,flat_ground_prior=flat_ground_prior);self.anchored=True
 def commit(self,program,*,initial_supervision_xy):
  if not self.anchored or self.failed:raise ValueError('anchor required')
  try:
   self.collector.commit(program)
   envelope=json.dumps(dict(sequence_id=self.collector.sequence,online_state=json.loads(self.collector.snapshot),left_rgb_hex=self.rgb[0].hex(),right_rgb_hex=self.rgb[1].hex(),source=self.source),sort_keys=True,allow_nan=False).encode()
   self.journal.commit(envelope,self.collector.proposal.raw_targets)
   self.journal.append(dict(type='initial_supervision',time_s=self.collector.proposal.time_s,world_xy=initial_supervision_xy))
   self.collector.supervise(self.collector.proposal.time_s,initial_supervision_xy)
  except BaseException:self.failed=True;self.journal.close();raise
 def step(self,execute):
  try:
   def callback(request):
    feedback=execute(request)
    self.journal.append(dict(type='execution_feedback',index=self.journal.index,feedback=feedback))
    # Actual returned command and clock, never substitute requested values.
    self.collector.consume(feedback['request_time'],feedback['actual_raw_pair'])
    self.collector.supervise(feedback['end_time'],feedback['end_world_xy'])
   self.journal.consume(callback)
  except BaseException:self.failed=True;self.journal.close();raise
 def qualify(self):
  if self.failed:return dict(scorable=False,reason='execution_interrupted',trusted_collector_verified=False)
  return self.collector.qualify()
