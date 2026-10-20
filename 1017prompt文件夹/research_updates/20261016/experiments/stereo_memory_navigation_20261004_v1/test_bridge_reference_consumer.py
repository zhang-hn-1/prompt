from types import SimpleNamespace
import numpy as np
import bridge_reference_consumer as m

def geometry():return dict(retention_necessary_screen_pass=False,raster_clearance_pass=True,suffix_clearance_m=.62,bridge_clearance_m=.58,projection_distance_m=.125,remaining_arc_m=1.2,reject_reasons=['continuous_observed_hazard_clearance'])
class Base:
 def __init__(self):
  self.plan=object();self.last_plan=0.;self.reference_installed_time=.1;self.local_reference_version=3;self.plan_meta={'status':'success'};self.calls=0;self.reference_decision=None
  self.grid=SimpleNamespace(labels=np.full((2,2),-1),origin=np.array([0,0]),resolution=.1)
  self.tracker=SimpleNamespace(_reference_xy=np.array([[0,0],[1,0]]),_path_index=2,_active_drive_direction='forward',_pending_drive_direction=None,_stopped_cycles=0,_off_reference_cycles=0,_failure_subreason=None,_braking_state='tracking',mapping_records=[])
 def plan_boundary(self,*args):self.calls+=1;return True,{'decision':'original'}
def test_alias_retains_existing_reference_and_tracker_without_replan(monkeypatch):
 monkeypatch.setattr(m,'reference_geometry_status',lambda *a:geometry());p=m.with_bridge_consumer(Base)();old=p.plan
 created,d=p.plan_boundary(np.array([0.,0.]),0.,np.array([1.,0.]),20.)
 assert not created and p.calls==0 and p.plan is old and p.local_reference_version==3 and p.reference_installed_time==.1 and p.last_plan==20.
 assert d['state_before']==d['state_after'] and d['decision']=='retain_raster_consistent_current_bridge' and d['guard']['retention_necessary_screen_pass']==False
def test_original_transaction_once_if_future_screen_invalid(monkeypatch):
 g=geometry();g['suffix_clearance_m']=.59;monkeypatch.setattr(m,'reference_geometry_status',lambda *a:g);p=m.with_bridge_consumer(Base)()
 created,d=p.plan_boundary(np.array([0.,0.]),0.,np.array([1.,0.]),20.)
 assert created and d['decision']=='original' and p.calls==1
