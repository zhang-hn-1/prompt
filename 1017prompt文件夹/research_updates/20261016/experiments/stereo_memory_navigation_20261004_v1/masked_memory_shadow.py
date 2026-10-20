"""Current-only consumer on same live CAD-filtered sources; never executes."""
from masked_source_pair import ReadOnlyMaskShadow
class ReadOnlyCurrentMemoryShadow(ReadOnlyMaskShadow):
 def consume(self,source,packet,observation,goal):
  row=super().consume(source,packet,observation,goal);row.update(source_filter='dynamic_current_joint_CAD',memory_role='current_source_only');return row
