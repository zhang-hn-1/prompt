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
from causal_source_delivery_memory import CausalSourceDeliveryMemory
H=Path(__file__).resolve().parent;D=H/'live_async_stereo_navigation_run_v1';protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));native=json.loads((D/'native_predictions.json').read_text());imu=json.loads((D/'native_imu_packets.json').read_text());source=json.loads((D/'source_rows.json').read_text());rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];zero=np.zeros((480,640,3),np.uint8)
# Raw images are unused by this traditional tracker: depth/RGB were already consumed online into immutable points.
def run(stop,poison):
 states=copy.deepcopy(native);packets=copy.deepcopy(imu);comparisons=copy.deepcopy(rows)
 if poison:
  for v in states[stop*5+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  for v in packets[stop*5+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  for v in comparisons[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:states=states[:stop*5+1];packets=packets[:stop*5+1];comparisons=comparisons[:stop+1]
 policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current');policy.reset(EpisodeContext('live_async_stereo_navigation',False,None,protocol.protocol_id));init=np.asarray(states[0]['position_world'])[:2];policy.grid=PartialPointRiskGrid(init-20,10);nav=EstimatedNavigationObservation();m=CausalSourceDeliveryMemory();out=[];cursor=0
 for i in range(stop+1):
  n=states[i*5];t=n['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(n['position_world'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=n['position_world'],root_rotation_world=n['rotation_world'],velocity_world_m_s=n['velocity_world_m_s']);observation,goal=nav.build(original,pose,packets[i*5]['gyro_body_rad_s'],[2.2,.15,.45])
  while cursor<len(source) and source[cursor]['available_frame']<=i:
   s=source[cursor];birth=s['frame'];assert s['available_time_s']<=t
   a=np.load(D/'partial_observations'/f'frame_{birth:03d}.npz')
   m.deliver(dict(frame=birth,time_s=s['time_s'],world=a['world'],query_pixel_id=a['query_pixel_id'],partner_pixel_id=a['partner_pixel_id'],input_sha256=s['provenance']),available_time_s=s['available_time_s'],now=s['available_time_s']);cursor+=1
  snap=m.snapshot(t);risk=dict(frame=snap['source_frame'],time_s=snap['source_time_s'],records=snap['records']);current=dict(frame=i,time_s=t,root_xyz=np.asarray(n['position_world']),points_world=np.empty((0,3)))
  if i<140:policy.grid.set_source(risk);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t);observe_clock_without_act(policy,current,t)
  else:
   policy.set_source_packet(risk);policy.set_current_packet(current);action=policy.act(observation,goal);np.testing.assert_array_equal(action,actual[i]);np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy),comparisons[i]['installed_reference_xy_after_act']);out.append(np.asarray(action))
 return np.asarray(out)
full=run(len(rows)-1,False)
checks=[]
for cutoff in sorted(set([min(145,len(rows)-2),max(140,len(rows)//2),len(rows)-2])):
 prefix=run(cutoff,False);original_load=np.load;opened=[]
 def guarded_load(file,*args,**kwargs):
  path=Path(file)
  if path.parent.name in ('history','partial_observations'):
   frame=int(path.stem.split('_')[-1]);assert frame<=cutoff,'future source/history file opened'
   if path.parent.name=='partial_observations':
    availability=next(r['available_frame'] for r in source if r['frame']==frame);assert availability<=cutoff,'captured but unavailable source opened'
   opened.append(str(path))
  return original_load(file,*args,**kwargs)
 try:
  np.load=guarded_load;poison=run(cutoff,True)
 finally:np.load=original_load
 np.testing.assert_array_equal(prefix,full[:len(prefix)]);np.testing.assert_array_equal(poison,prefix)
 checks.append(dict(cutoff_frame=cutoff,prefix_actions=len(prefix),future_payload_poison_and_truncation_exact=True,guarded_past_file_reads=len(opened)))
a=dict(decision='PASS_FULL_REQUEST_RECONSTRUCTION_AND_THREE_PREFIX_CAUSALITY_ONLY',all_navigation_requests_exact=len(full),cutoffs=checks,future_source_history_file_reads_blocked=True,past_but_unavailable_source_reads_blocked=True,immutable_birth_and_delivery_clocks=True,dummy_RGB_unused_by_tracker=True,not_another_online_run=True,source_and_nav_gates_separate=True,whole_complete=False)
(H/'live_async_stereo_policy_causal_assessment_v2.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
