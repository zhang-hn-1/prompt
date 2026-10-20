from pathlib import Path
import json
from types import SimpleNamespace as S
import numpy as np

def test_reference_installs_once_retains_complete_geometry_and_control_state(monkeypatch):
 import fixed_reference_pacing_harness_v1 as h
 frozen=json.loads((Path(__file__).parent/'straight_pacing_fixed_reference_v1.json').read_text());calls=[]
 def install(tracker,plan):calls.append(plan);tracker._reference_xy=np.array([[w.x_m,w.y_m] for w in plan.waypoints])
 monkeypatch.setattr(h,'install_local_reference',install)
 class B:pass
 x=h.with_frozen_reference(B,frozen)();x.plan=None;x.tracker=S(_path_index=0,_active_drive_direction='forward',_pending_drive_direction=None,_stopped_cycles=0,_off_reference_cycles=0,_failure_subreason=None,_braking_state='tracking',mapping_records=[])
 first,_=x.plan_boundary(None,None,None,6.);x.tracker._path_index=3;x.tracker._stopped_cycles=2
 second,d=x.plan_boundary(None,None,None,7.)
 assert first and not second and len(calls)==1 and len(x.plan.waypoints)==len(frozen['waypoints'])
 assert x.tracker._path_index==3 and x.tracker._stopped_cycles==2 and x.reference_installed_time==6.
 assert x.local_reference_version==1 and d['decision']=='fixed_retain'
