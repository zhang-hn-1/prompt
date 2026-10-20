import bootstrap
from bootstrap import D,W
import json,numpy as np
from unittest.mock import patch
from benchmark.policies import EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT
from prior_rgb_navigation import PriorRGBPolicy
from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
from command_domain_cap import with_command_domain_cap
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from viewpoint_confirmation import ViewpointConfirmationGrid
from reserve import with_replanning_reserve,reserve_curve,reserve_screen,ReserveGrid
from guarded_reference import boundary_state
from reference_continuity_audit import reference_geometry_status
R=W/'experiments/source_confidence_calibration_20261012_v1/data_e1_run';s=json.loads((R/'local_plans/frame_060.json').read_text());z=np.load(R/'maps/frame_060.npz');wp=s['waypoints'][0];xy=np.array([wp['x_m'],wp['y_m']]);yaw=wp['yaw_rad'];goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
proto=load_action_protocol(DEFAULT_CONTRACT.resolve());Base=with_command_domain_cap(with_cusp_coast_acquisition(with_first_primitive_direction(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(PriorRGBPolicy)))))
C=with_replanning_reserve(Base);obj=C(proto,memory_mode='current');obj.reset(EpisodeContext('check',False,None,proto.protocol_id));obj.grid=ViewpointConfirmationGrid(z['origin'],10,static_scene_assumption=True);obj.grid.labels=z['labels'].copy();before=obj.grid.labels.copy()
changed,_=obj.plan_boundary(xy,yaw,goal,6.);assert changed and obj.plan is not None and obj.local_reference_version==1
assert obj.grid._goal is not None;assert obj.plan_meta['generated_reference_reserve_screen']['pass'];np.testing.assert_array_equal(obj.grid.labels,before)
ref=obj.tracker._reference_xy.copy();ver=obj.local_reference_version;changed,_=obj.plan_boundary(xy,yaw,goal,6.5);assert not changed and obj.local_reference_version==ver;np.testing.assert_array_equal(ref,obj.tracker._reference_xy)
try:ReserveGrid(obj.grid).plan(xy,np.array(goal)+[.01,0]);raise AssertionError('fixed goal bypass')
except ValueError:pass
# Source clock, masks and map are outside this intervention; no empty plan install.
st=boundary_state(obj.tracker)
with patch('reserve.reserve_curve',return_value=(None,dict(status='unreachable',unknown_fraction=1))):
 obj.plan=None;changed,_=obj.plan_boundary(xy,yaw,goal,7.);assert changed and obj.plan is None;assert st==boundary_state(obj.tracker)
np.testing.assert_array_equal(obj.grid.labels,before)
obj.current_first_direction='forward';obj.plan_boundary(xy,yaw,goal,7.5);assert obj.plan is not None;assert all(w.main_command_rad_s>0 for w in obj.plan.waypoints[:2]);assert obj.first_direction_query['minimum_initial_run_primitives']==2
(D/'mro_checks.json').write_text(json.dumps(dict(pass_gate=True,checks=['actualMRO','single_install','retain','fixed_goal_reject','no_route_state_preserved','map_immutable','measured_first_direction_prefix','continuous_generated_guard'],MRO=[c.__name__ for c in C.__mro__]),indent=2)+'\n');print('8 actualMRO checks passed')
