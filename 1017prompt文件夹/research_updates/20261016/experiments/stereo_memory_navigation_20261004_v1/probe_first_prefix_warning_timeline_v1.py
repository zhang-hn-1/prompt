from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
import minimum_prefix_transition_planning_v1 as search
from execution_domain_four_point_source_v2 import supported_requests,supported_curvature
from execution_domain_four_point_consumer_v2 import planning_clearance
from continuous_cell_tube_v1 import continuous_cell_tube_screen
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());res=[]
commands=[r for r in supported_requests() if r[0]>0]
for i in [471,486,501]:
 s=states[i];r=rows[i];assert r['guard_measured_this_call'] and r['tracker_state_after_act']['active_direction']=='forward';m=np.load(D/'maps'/f'frame_{i:03d}.npz');g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'].copy();occ=(distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution<planning_clearance(g)-1e-8).astype(np.uint8);R=Rotation.from_quat(np.asarray(s['quaternion'])[[1,2,3,0]]).as_matrix();start=np.r_[s['position'][:2],np.arctan2(R[1,0],R[0,0])];entries=[]
 for c1 in commands:
  one=search.sample_motion_primitive(start_pose=start,signed_arc_length_m=.3,curvature_1_m=supported_curvature(*c1),sample_spacing_m=.05)
  for c2 in commands:
   two=search.sample_motion_primitive(start_pose=one[-1],signed_arc_length_m=.3,curvature_1_m=supported_curvature(*c2),sample_spacing_m=.05);path=np.vstack([one,two[1:]]);sampled=search.primitive_is_collision_free(path,occupancy_grid=occ,origin_xy=g.origin,resolution_m=g.resolution);continuous=continuous_cell_tube_screen(g,dict(bridge_xy=path[:1,:2],suffix_xy=path[:,:2]));entries.append(dict(requests=[list(c1),list(c2)],sampled_pass=sampled,continuous_pass=continuous['pass'],continuous_minimum_m=continuous['minimum_continuous_cell_clearance_m']))
 res.append(dict(frame=i,time_s=r['time_s'],current_original_reference_decision=r['reference_decision']['decision'],current_reference_tube_screen=r['reference_decision']['tube_viability_screen'],current_raw_request=r['request'],existing_first_direction_query=r['first_primitive_direction_query'],two_prefix_sampled_feasible=sum(e['sampled_pass'] for e in entries),two_prefix_continuous_feasible=sum(e['continuous_pass'] for e in entries),hypothetical_current_state_commands=entries))
out=dict(protocol=json.loads((H/'first_prefix_warning_timeline_protocol_v1.json').read_text()),rows=res,no_new_controller=True,no_physical_counterfactual_truth=True);(D/'first_prefix_warning_timeline_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps([dict(frame=r['frame'],reference_decision=r['current_original_reference_decision'],sampled=r['two_prefix_sampled_feasible'],continuous=r['two_prefix_continuous_feasible']) for r in res]))
