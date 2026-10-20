import numpy as np
from endpoint_tail_retention_v1 import endpoint_tail_screen

def guard(reasons):return dict(reject_reasons=list(reasons),remaining_arc_m=.4,retention_necessary_screen_pass=not reasons)
def test_safe_short_tail_preserved_until_existing_goal_distance():
 d=endpoint_tail_screen(guard(['remaining_arc_under_lookahead']),[.6,0],[[0,0],[1,0]],True,.2)
 assert d['retention_necessary_screen_pass'] and d['endpoint_tail_intervened'];assert d['endpoint_distance_m']==.4
 assert not endpoint_tail_screen(guard(['remaining_arc_under_lookahead']),[.9,0],[[0,0],[1,0]],True,.2)['retention_necessary_screen_pass']
def test_hazard_or_ineligible_never_bypassed():
 for reasons in (['remaining_arc_under_lookahead','continuous_observed_hazard_clearance'],['remaining_arc_under_lookahead','original_raster_clearance'],['projection_distance_over_0p5']):
  d=endpoint_tail_screen(guard(reasons),[.6,0],[[0,0],[1,0]],True,.2);assert not d['retention_necessary_screen_pass'];assert d['reject_reasons']==reasons
 assert not endpoint_tail_screen(guard(['remaining_arc_under_lookahead']),[.6,0],[[0,0],[1,0]],False,.2)['retention_necessary_screen_pass']
def test_guard_input_unchanged_and_duplicate_or_nonfinite_rejected():
 import pytest
 g=guard(['remaining_arc_under_lookahead']);endpoint_tail_screen(g,[.6,0],[[0,0],[1,0]],True,.2);assert g['reject_reasons']==['remaining_arc_under_lookahead']
 with pytest.raises(ValueError):endpoint_tail_screen(g,[np.nan,0],[[0,0],[1,0]],True,.2)

def test_composed_boundary_keeps_first_primitive_globals_and_old_state():
 from types import SimpleNamespace
 from endpoint_tail_retention_v1 import with_first_endpoint_tail
 from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
 from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
 class EmptyBase:pass
 cls=with_first_primitive_direction(with_first_endpoint_tail(with_continuous_cell_tube_after_measured_reversal(EmptyBase)))
 obj=object.__new__(cls);ref=np.array([[0.,0.],[1.,0.]])
 tracker=SimpleNamespace(_path_index=0,_active_drive_direction='forward',_pending_drive_direction=None,_stopped_cycles=0,_off_reference_cycles=0,_failure_subreason=None,_braking_state='tracking',mapping_records=[],_reference_xy=ref,_reference=(SimpleNamespace(drive_direction='forward'),)*2,_context=None)
 obj.tracker=tracker;obj.plan=object();obj.local_reference_version=1;obj.grid=SimpleNamespace(labels=np.full((40,40),-1,dtype=np.int8),origin=np.array([-2.,-2.]),resolution=.1);obj.tube_viability_armed=False;obj.current_first_direction=None;obj.first_direction_query=None;obj.reference_installed_time=0.
 created,decision=obj.plan_boundary(np.array([.6,0.]),0.,np.array([4.,0.]),1.)
 assert created is False and decision['decision']=='retain';assert decision['state_before']==decision['state_after'];assert obj.local_reference_version==1;assert obj.first_direction_query is None;assert obj.endpoint_tail_last['endpoint_tail_intervened']
