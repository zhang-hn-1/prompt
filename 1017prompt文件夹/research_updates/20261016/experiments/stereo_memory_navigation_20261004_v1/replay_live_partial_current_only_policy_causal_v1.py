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
H=Path(__file__).resolve().parent;D=H/'live_partial_current_only_navigation_run_v1';protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));native=json.loads((D/'native_predictions.json').read_text());imu=json.loads((D/'native_imu_packets.json').read_text());source={r['frame']:r for r in json.loads((D/'source_rows.json').read_text())};rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];zero=np.zeros((480,640,3),np.uint8)
# Raw images are unused by this traditional tracker: depth/RGB were already consumed online into immutable points.
def run(stop,poison):
 states=copy.deepcopy(native);packets=copy.deepcopy(imu);comparisons=copy.deepcopy(rows)
 if poison:
  for v in states[stop*5+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  for v in packets[stop*5+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  for v in comparisons[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:states=states[:stop*5+1];packets=packets[:stop*5+1];comparisons=comparisons[:stop+1]
 policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current');policy.reset(EpisodeContext('live_partial_current_only_navigation',False,None,protocol.protocol_id));init=np.asarray(states[0]['position_world'])[:2];policy.grid=PartialPointRiskGrid(init-20,10);nav=EstimatedNavigationObservation();m=TentativePointMemory();out=[]
 for i in range(stop+1):
  n=states[i*5];t=n['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(n['position_world'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=n['position_world'],root_rotation_world=n['rotation_world'],velocity_world_m_s=n['velocity_world_m_s']);observation,goal=nav.build(original,pose,packets[i*5]['gyro_body_rad_s'],[2.2,.15,.45])
  if i%5==0:
   a=np.load(D/'partial_observations'/f'frame_{i:03d}.npz');s=source[i];m.update(dict(frame=i,time_s=t,world=a['world'],query_pixel_id=a['query_pixel_id'],partner_pixel_id=a['partner_pixel_id'],input_sha256=s['provenance']),t)
  s=source[(i//5)*5];risk=dict(frame=s['frame'],time_s=s['time_s'],records=[r for r in m.at(t)['records'] if r['time_s']==s['time_s']]);current=dict(frame=i,time_s=t,root_xyz=np.asarray(n['position_world']),points_world=np.empty((0,3)))
  if i<140:policy.grid.set_source(risk);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t);observe_clock_without_act(policy,current,t)
  else:
   policy.set_source_packet(risk);policy.set_current_packet(current);action=policy.act(observation,goal);np.testing.assert_array_equal(action,actual[i]);np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy),comparisons[i]['installed_reference_xy_after_act']);out.append(np.asarray(action))
 return np.asarray(out)
full=run(len(rows)-1,False);cutoff=min(145,len(rows)-1);prefix=run(cutoff,False);poison=run(cutoff,True);np.testing.assert_array_equal(prefix,full[:len(prefix)]);np.testing.assert_array_equal(poison,prefix)
a=dict(decision='PASS_CURRENT_PAST_POLICY_RECONSTRUCTION_AND_FIRST_NAV_PREFIX_CAUSALITY_ONLY',all_navigation_requests_exact=len(full),cutoff_frame=cutoff,prefix_actions=len(prefix),future_native_and_feedback_policy_log_payload_poison_exact=True,future_source_files_not_opened_in_prefix=True,dummy_RGB_unused_by_traditional_tracker=True,not_online_interface_replacement=True,no_nav_or_source_gate_rescue=True,whole_complete=False);(H/'live_partial_current_only_policy_causal_assessment_v1.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
