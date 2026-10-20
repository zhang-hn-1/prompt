import copy,math,pytest
from raster_consistent_bridge import bridge_screen

def guard():return dict(retention_necessary_screen_pass=False,raster_clearance_pass=True,suffix_clearance_m=.6196776177,bridge_clearance_m=.5795600467,projection_distance_m=.1251,remaining_arc_m=1.2086,reject_reasons=['continuous_observed_hazard_clearance'])
def test_current_bridge_alias_keeps_future_screen_and_original_record():
 g=guard();saved=copy.deepcopy(g);r=bridge_screen(dict(time_s=20.,guard=g),20.,.1)
 assert r['candidate_retain'] and not r['original_retain'] and r['body_cell_clearance_lower_bound_m']>.4
 assert r['bridge_bound_m']==pytest.approx(.6-.1*math.sqrt(2)/2) and g==saved
@pytest.mark.parametrize('key,value',[('suffix_clearance_m',.59),('bridge_clearance_m',.51),('raster_clearance_pass',False),('projection_distance_m',.7),('remaining_arc_m',.4)])
def test_no_future_raster_body_or_tracking_relaxation(key,value):
 g=guard();g[key]=value;assert not bridge_screen(dict(time_s=20.,guard=g),20.,.1)['candidate_retain']
def test_coarse_grid_is_outside_physical_protection_domain():
 assert not bridge_screen(dict(time_s=20.,guard=guard()),20.,.3)['candidate_retain']
def test_future_guard_rejected_before_payload():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('futureguardpayload')
   return 21.
 with pytest.raises(ValueError):bridge_screen(Bomb(),20.,.1)
