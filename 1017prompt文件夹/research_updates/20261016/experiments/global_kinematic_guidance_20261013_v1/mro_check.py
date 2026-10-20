import bootstrap
from bootstrap import D,W,H
import json,types,numpy as np
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
from global_guidance import with_global_guidance,global_curve,original,search
from viewpoint_confirmation import ViewpointConfirmationGrid
R=W/'experiments/source_confidence_calibration_20261012_v1/data_e1_run';s=json.loads((R/'local_plans/frame_060.json').read_text());z=np.load(R/'maps/frame_060.npz');p=s['waypoints'][0];xy=np.array([p['x_m'],p['y_m']]);yaw=p['yaw_rad'];goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
proto=load_action_protocol(DEFAULT_CONTRACT.resolve());C=with_command_domain_cap(with_cusp_coast_acquisition(with_global_guidance(with_first_primitive_direction(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(PriorRGBPolicy))))));obj=C(proto,memory_mode='current');obj.reset(EpisodeContext('qualification',False,None,proto.protocol_id));grid=ViewpointConfirmationGrid(z['origin'],10,static_scene_assumption=True);grid.labels=z['labels'].copy();grid.plan=types.MethodType(lambda self,x,g:(np.array(s['plan_meta']['route_xy']), 'unknown_exploration'),grid);obj.grid=grid
changed,decision=obj.plan_boundary(xy,yaw,goal,6.);assert changed and obj.local_reference_version==1;assert not obj.plan_meta['global_kinematic']['fallback'];assert obj.first_direction_query['minimum_initial_run_primitives']==0;assert obj.tracker._mapping_count==0 if hasattr(obj.tracker,'_mapping_count') else True
ref=obj.tracker._reference_xy.copy();version=obj.local_reference_version
changed,d=obj.plan_boundary(xy,yaw,goal,6.5);assert not changed and obj.local_reference_version==version;np.testing.assert_array_equal(obj.tracker._reference_xy,ref)
# Failed global query returns byte-identical original plan; no synthetic free/map change.
args=(grid,xy,yaw,goal,obj.tracker.drive_model);kw=dict(initial_direction=None,first_primitive_direction=None);b,bm=original(*args,**kw)
with patch('global_guidance.search.plan_kinodynamic_path',return_value=search.KinodynamicPlanResult('node_budget_exhausted',(),3000,0.,0)):
 # The inherited baseline calls same search module; isolate original result to test fallback transaction.
 with patch('global_guidance.original',return_value=(b,bm)):
  q,qm=global_curve(*args,**kw);assert q is b and qm['global_kinematic']['fallback']
# Measured forward prefix propagates through real boundary, not a global override.
obj.plan=None;obj.current_first_direction='forward';obj.plan_boundary(xy,yaw,goal,7.);full=obj.plan_meta['global_kinematic']['full_waypoints'];assert all(w['main_command_rad_s']>0 for w in full[:2]);assert obj.first_direction_query['minimum_initial_run_primitives']==2
r=dict(pass_gate=True,checks=['actual_policy_MRO','single_install','retain_no_rebuild','exact_failure_fallback','first_direction_two_primitives','current_maps_only'],MRO=[c.__name__ for c in C.__mro__]);(D/'mro_checks.json').write_text(json.dumps(r,indent=2)+'\n');print(r)
