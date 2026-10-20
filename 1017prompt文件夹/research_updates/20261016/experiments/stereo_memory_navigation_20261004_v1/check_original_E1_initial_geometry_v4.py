from pathlib import Path
import json,hashlib,numpy as np
from partial_point_risk_grid import PartialPointRiskGrid
from tentative_point_memory import TentativePointMemory
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points,local_curve_for_direction_tube
from command_domain_cap import with_command_domain_cap
from scripts.run_pointgoal_benchmark import load_action_protocol
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT
from benchmark.policies import EpisodeContext
from types import SimpleNamespace
H=Path(__file__).resolve().parent;D=H/'simple_E1_partial_source_probe_run_v1';z=np.load(D/'partial_observations/frame_000.npz');pose=np.load(D/'known_poses/frame_000.npz');hist=np.load(D/'history/frame_000.npz');initial=np.asarray(pose['root_position_world_m']);mem=TentativePointMemory();mem.update(dict(frame=0,time_s=0.,world=z['world'],query_pixel_id=z['query_pixel_id'],partner_pixel_id=z['partner_pixel_id'],input_sha256=json.loads((D/'source_rows.json').read_text())[0]['provenance']),0.);snapshot=mem.at(0.)
protocol=load_action_protocol(DEFAULT_CONTRACT.resolve());policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current');policy.reset(EpisodeContext('original_E1_initial_consumer_CPU_only',False,None,protocol.protocol_id));policy.grid=PartialPointRiskGrid(initial[:2]-20,10);policy.set_source_packet(dict(frame=0,time_s=0.,records=snapshot['records']));policy.set_current_packet(dict(frame=0,time_s=0.,points_world=np.empty((0,3)),root_xyz=initial));obs=SimpleNamespace(position_world_xy=initial[:2],yaw_world_rad=0.,proprio_history=hist['proprio_history'],quaternion_world_wxyz_history=hist['quaternion_world_wxyz_history'],action_history=hist['action_history'])
request=policy.act(obs,np.array([3.300000005960465-initial[0],5.960465e-9-initial[1]]))
assert not np.any(request) and policy.plan is None and policy.plan_meta=={}
geometry_plan,geometry_meta=local_curve_for_direction_tube(policy.grid,initial[:2],0.,np.array([3.300000005960465,5.960465e-9]),policy.tracker.drive_model,initial_direction=None)

r=dict(decision='PASS_INITIAL_GEOMETRY_AND_FROZEN_SETTLING_SCOPE' if geometry_plan is not None else 'REJECT_INITIAL_GEOMETRY',runtime_plan_at_zero_expected_none=True,frozen_settling_s=6.,points=len(z['world']),hazard_cells=int((policy.grid.labels==1).sum()),free_cells=int((policy.grid.labels==0).sum()),source_frame=0,initial_prior_not_free=True,request=request.tolist(),plan_status=None if geometry_plan is None else geometry_plan.status,plan_meta=geometry_meta,not_live_runtime=True,not_navigation=True,not_repeated_policy_act=True,no_depth_truth_labels_in_consumer=True,whole_research_complete=False)
(H/'original_E1_initial_consumer_CPU_assessment_v4.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
