from pathlib import Path
import json,copy,numpy as np
from benchmark.policies import PolicyObservation,EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from partial_point_risk_grid import PartialPointRiskGrid
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from estimated_navigation_observation import EstimatedNavigationObservation
from observation_clock_adapter import observe_clock_without_act
from tentative_point_memory import TentativePointMemory
from partial_surface_memory import PartialSurfaceMemory
H=Path(__file__).resolve().parent;D=H/'live_generic_height_navigation_run_v1';protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));native=json.loads((D/'native_predictions.json').read_text());imu=json.loads((D/'native_imu_packets.json').read_text());source={r['frame']:r for r in json.loads((D/'source_rows.json').read_text())};rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];zero=np.zeros((480,640,3),np.uint8)
# Raw images are unused by this traditional tracker: depth/RGB were already consumed online into immutable points.
def run(stop,poison,colour=False):
 states=copy.deepcopy(native);packets=copy.deepcopy(imu);comparisons=copy.deepcopy(rows)
 if poison:
  for v in states[stop*5+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  for v in packets[stop*5+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  for v in comparisons[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:states=states[:stop*5+1];packets=packets[:stop*5+1];comparisons=comparisons[:stop+1]
 policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current');policy.reset(EpisodeContext('live_generic_height_navigation',False,None,protocol.protocol_id));init=np.asarray(states[0]['position_world'])[:2];policy.grid=PartialPointRiskGrid(init-20,10);nav=EstimatedNavigationObservation();m=TentativePointMemory();surfaces=PartialSurfaceMemory();out=[]
 for i in range(stop+1):
  n=states[i*5];t=n['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(n['position_world'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=n['position_world'],root_rotation_world=n['rotation_world'],velocity_world_m_s=n['velocity_world_m_s']);observation,goal=nav.build(original,pose,packets[i*5]['gyro_body_rad_s'],[2.2,.15,.45])
  if i%5==0:
   s=source[i]
   if colour:
    packet=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');masks=np.load(D/'partial_class_masks'/f'frame_{i:03d}.npz');a=surfaces.update(packet,masks['left'],masks['right'],t,s['provenance'])
   else:a=np.load(D/'partial_observations'/f'frame_{i:03d}.npz')
   m.update(dict(frame=i,time_s=t,world=a['world'],query_pixel_id=a['query_pixel_id'],partner_pixel_id=a['partner_pixel_id'],input_sha256=s['provenance']),t)
  s=source[(i//5)*5];risk=dict(frame=s['frame'],time_s=s['time_s'],records=m.at(t)['records']);current=dict(frame=i,time_s=t,root_xyz=np.asarray(n['position_world']),points_world=np.empty((0,3)))
  if i<140:policy.grid.set_source(risk);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t);observe_clock_without_act(policy,current,t)
  else:
   policy.set_source_packet(risk);policy.set_current_packet(current);action=policy.act(observation,goal);if not colour:
    np.testing.assert_array_equal(action,actual[i]);np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy),comparisons[i]['installed_reference_xy_after_act'])
   elif not np.array_equal(action,actual[i]):
    evidence=dict(first_request_divergence_frame=i,time_s=t,actual_generic_request=actual[i].tolist(),counterfactual_colour_request=np.asarray(action).tolist(),generic_realized_reference=comparisons[i]['installed_reference_xy_after_act'],colour_reference=np.asarray(policy.tracker._reference_xy).tolist(),colour_policy_row=policy.records[-1],only_source_selector_changed=True,stop_before_action_effect=True,no_alternate_future_physics_label=True)
    (H/'generic_height_same_prefix_source_first_divergence_v1.json').write_text(json.dumps(evidence,indent=2)+'\n');return evidence
   out.append(np.asarray(action))
 return np.asarray(out)
run(len(rows)-1,False,False)
a=run(len(rows)-1,False,True);print(json.dumps({k:v for k,v in a.items() if k not in ['generic_realized_reference','colour_reference','colour_policy_row']},indent=2))
