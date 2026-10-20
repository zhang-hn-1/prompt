"""Persist policy-only diagnostics before native simulator shutdown."""
import json
from pathlib import Path

def wrap_runtime(base_class,policy_getter,output):
 class LoggingRuntime(base_class):
  def close(self,*args,**kwargs):
   policy=policy_getter()
   if policy is not None and hasattr(policy,'mapping_records') and hasattr(self,'bank_episode_id'):
    dest=Path(output);dest.mkdir(parents=True,exist_ok=True)
    (dest/('mapping_'+self.bank_episode_id+'.json')).write_text(json.dumps(policy.mapping_records))
   return super().close(*args,**kwargs)
 return LoggingRuntime
