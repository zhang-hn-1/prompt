import numpy as np
from types import SimpleNamespace
from bounded_forward_prefix_selection_v1 import choose_prefix
from reference_continuity_audit import reference_geometry_status

def grid(label=-1):return SimpleNamespace(labels=np.full((121,121),label,np.int8),origin=np.array([-6.,-6.]),resolution=.1)

def test_unchanged_guard_excludes_single_step_and_short_expiry():
 g=grid();assert not reference_geometry_status([0,0],[[0,0],[.3,0]],g.labels,g.origin,.1)['retention_necessary_screen_pass']
 assert reference_geometry_status([0,0],[[0,0],[.3,0],[.6,0]],g.labels,g.origin,.1)['retention_necessary_screen_pass']
 assert not reference_geometry_status([.11,0],[[0,0],[.3,0],[.6,0]],g.labels,g.origin,.1)['retention_necessary_screen_pass']

def test_only_existing_two_step_compatible_prefix_and_no_free_space_claim():
 g=grid();before=g.labels.copy();c,trace=choose_prefix([0,0],0,[1,0],g,lambda main,aux:0.)
 assert len(trace)==12 and len(c['requests'])==2 and c['guard']['retention_necessary_screen_pass']
 assert len([x for x in trace if x['one_step'] and x['selected_eligible']])==0
 assert c['partial_reference'] and not c['certified_safe'] and not c['observed_free'] and not c['old_local_target_reached']
 assert abs(c['goal_axis_progress_m']-.6)<1e-12;np.testing.assert_array_equal(g.labels,before)

def test_blocked_or_negative_goal_progress_returns_no_override():
 assert choose_prefix([0,0],0,[1,0],grid(1),lambda m,a:0.)[0] is None
 assert choose_prefix([0,0],0,[-1,0],grid(),lambda m,a:0.)[0] is None

def test_actual_frozen_nominal_source_synthetic_root_only():
 from execution_domain_four_point_source_v2 import supported_curvature
 c,trace=choose_prefix([0,0],0,[1,0],grid(),supported_curvature)
 assert c is not None and len(c['requests'])==2 and len(trace)==12
 assert abs(c['poses'][-1][1])>1e-5
 assert all(m==.5 and a in (0.,-np.pi/6,np.pi/6) for x in trace for m,a in x['requests'])
