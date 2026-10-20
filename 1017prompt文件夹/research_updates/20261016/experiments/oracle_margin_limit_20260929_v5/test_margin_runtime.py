import dataclasses,json
from pathlib import Path
from margin_runtime import with_planning_margin
@dataclasses.dataclass(frozen=True)
class Config:
 safety_margin_m:float=.1
 ball_radius_m:float=.4
 auxiliary_velocity_limit_rad_s:float=.3
class Base:
 def __init__(self,config):self.config=config;self._episode=None;self._contact_available=True
 def _capture(self):return 'unchanged observation'
 def close(self,*args,**kwargs):return 'closed'
def test_changes_planning_margin_only(tmp_path):
 original=Config();r=with_planning_margin(Base,.2,tmp_path)(original)
 assert original.safety_margin_m==.1
 assert dataclasses.asdict(r.config)==dict(safety_margin_m=.2,ball_radius_m=.4,auxiliary_velocity_limit_rad_s=.3)
 assert r._capture()=='unchanged observation'
 assert r.close()=='closed'
 row=json.loads((tmp_path/'runtime_audit_unknown.json').read_text())
 assert row['contact_available_samples']==[True]
 assert row['effective_config']['ball_radius_m']==.4
