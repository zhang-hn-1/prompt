"""Read-only timing around exact cached projection, no decision input."""
import time
from static_base_cached_self_mask_v2 import StaticBaseCachedSelfMask
class TimedStaticBaseSelfMask(StaticBaseCachedSelfMask):
 def project(self,*args,**kwargs):
  begin=time.monotonic();out=super().project(*args,**kwargs);self.last_project_wall_s=time.monotonic()-begin;return out
