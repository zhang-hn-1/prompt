"""Isolated planning-inflation experiment; collision footprint is untouched."""
from dataclasses import replace,asdict
import json

def with_planning_margin(base,margin,output):
 class MarginRuntime(base):
  def __init__(self,config):
   self.contact_available_samples=[]
   super().__init__(replace(config,safety_margin_m=margin))
  def _capture(self):
   result=super()._capture()
   self.contact_available_samples.append(bool(getattr(self,'_contact_available',False)))
   return result
  def close(self,*args,**kwargs):
   output.mkdir(parents=True,exist_ok=True)
   episode=getattr(getattr(self,'_episode',None),'episode_id','unknown')
   (output/f'runtime_audit_{episode}.json').write_text(json.dumps(dict(effective_config=asdict(self.config),contact_available_samples=self.contact_available_samples),default=str,indent=2))
   return super().close(*args,**kwargs)
 return MarginRuntime
