"""Fixed75-tick synthetic latency; stopbefore first counterfactual physical effect."""
from pathlib import Path
import json,copy,hashlib,datetime,numpy as np
from benchmark.policies import PolicyObservation,EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from partial_point_risk_grid import PartialPointRiskGrid
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from estimated_navigation_observation import EstimatedNavigationObservation
from observation_clock_adapter import observe_clock_without_act
from causal_source_delivery_memory import CausalSourceDeliveryMemory
H=Path(__file__).resolve().parent;D=H/'live_paired_texture_navigation_run_v1';O=H/'causal_delayed_source_delivery_replay_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');dt=float(np.float32(.02));delay=75*dt
manifest=D/'artifact_manifest_post_close.json'
for f,s in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==s,f
(O/'pre_replay_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256={str(p.resolve()):sha(p) for p in [Path(__file__),H/'causal_source_delivery_memory.py',H/'partial_point_risk_grid.py']},input_manifest_sha256=sha(manifest),synthetic_delay_s=delay,synthetic_delay_physics_ticks=75,no_measured_latency_fit=True,no_future_or_GT_actor_inputs=True),indent=2)+'\n')
contract=load_action_protocol(P/'configs/data_contract.yaml');native=json.loads((D/'native_predictions.json').read_text());imu=json.loads((D/'native_imu_packets.json').read_text());source={r['frame']:r for r in json.loads((D/'source_rows.json').read_text())};rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];zero=np.zeros((480,640,3),np.uint8)
def run(delayed,stop=None,poison=False):
 state=copy.deepcopy(native);packets=copy.deepcopy(imu);sr=copy.deepcopy(source);expected=copy.deepcopy(rows);limit=len(rows)-1 if stop is None else stop
 if poison:
  for a in state[(limit+1)*5:]:a.clear();a['FORBIDDEN_FUTURE']=None
  for a in packets[(limit+1)*5:]:a.clear();a['FORBIDDEN_FUTURE']=None
  for a in expected[limit+1:]:a.clear();a['FORBIDDEN_FUTURE']=None
  for f in sr:
   if f>140 and f*5*dt+delay>limit*5*dt:sr[f]={'FORBIDDEN_UNAVAILABLE':True}
 else:state=state[:limit*5+1];packets=packets[:limit*5+1];expected=expected[:limit+1]
 policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(contract,memory_mode='current');policy.reset(EpisodeContext('causal_delayed_source_interface',False,None,contract.protocol_id));policy.grid=PartialPointRiskGrid(np.asarray(state[0]['position_world'])[:2]-20,10);nav=EstimatedNavigationObservation();memory=CausalSourceDeliveryMemory();next_source=0;out=[];details=[];first=None;original_load=np.load;current_i=-1;opened=[]
 def guarded_load(file,*a,**kw):
  path=Path(file)
  if path.parent.name in ('history','partial_observations'):
   f=int(path.stem.split('_')[-1]);assert f<=current_i,'future acquisition file'
   if path.parent.name=='partial_observations':assert f*5*dt+(delay if delayed and f>140 else 0)<=current_i*5*dt,'unavailable source file'
   opened.append((current_i,path.parent.name,f))
  return original_load(file,*a,**kw)
 np.load=guarded_load
 try:
  for i in range(limit+1):
   current_i=i;n=state[i*5];t=n['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(n['position_world'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=n['position_world'],root_rotation_world=n['rotation_world'],velocity_world_m_s=n['velocity_world_m_s']);observation,goal=nav.build(original,pose,packets[i*5]['gyro_body_rad_s'],[2.2,.15,.45])
   while next_source<=i:
    available=next_source*5*dt+(delay if delayed and next_source>140 else 0)
    if available>t:break
    f=next_source;s=sr[f];a=np.load(D/'partial_observations'/f'frame_{f:03d}.npz');memory.deliver(dict(frame=f,time_s=f*5*dt,world=a['world'],query_pixel_id=a['query_pixel_id'],partner_pixel_id=a['partner_pixel_id'],input_sha256=s['provenance']),available_time_s=available,now=t);next_source+=5
   m=memory.snapshot(t);risk=dict(frame=m['source_frame'],time_s=m['source_time_s'],records=m['records']);current=dict(frame=i,time_s=t,root_xyz=np.asarray(n['position_world']),points_world=np.empty((0,3)))
   if i<140:policy.grid.set_source(risk);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t);observe_clock_without_act(policy,current,t)
   else:
    policy.set_source_packet(risk);policy.set_current_packet(current);action=policy.act(observation,goal);out.append(np.asarray(action));same=bool(np.array_equal(action,actual[i]));details.append(dict(frame=i,time_s=t,latest_delivered_source_frame=m['source_frame'],latest_acquisition_s=m['source_time_s'],latest_available_s=m['available_time_s'],source_age_s=t-m['source_time_s'],request_same_as_factual=same,request=action.tolist(),historical_points=m['historical_count'],current_points=m['current_count']))
    if not delayed:
     assert same;np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy),expected[i]['installed_reference_xy_after_act'])
    elif not same:first=i;break
 finally:np.load=original_load
 return np.asarray(out),details,first,opened
factual,_,_,_=run(False);candidate,details,first,opened=run(True);stop=first if first is not None else len(rows)-1;prefix,_,pf,_=run(True,stop=stop,poison=False);poison,_,xf,xopened=run(True,stop=stop,poison=True);np.testing.assert_array_equal(candidate,prefix);np.testing.assert_array_equal(candidate,poison);assert pf==xf==first
(O/'candidate_prefix_records.json').write_text(json.dumps(details,indent=2)+'\n');r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_DELIVERY_CLOCK_CAUSAL_INTERFACE_ONLY',factual_navigation_requests_and_references_exact=len(factual),synthetic_delay_s=delay,synthetic_delay_ticks=75,common_observer_prefix_preserved=True,first_candidate_request_difference_frame=first,physical_effect_not_simulated_at_index=None if first is None else first*5+1,candidate_evaluated_navigation_calls=len(candidate),future_and_unavailable_source_file_reads_blocked=True,future_payload_poison_and_truncation_prefix_exact=True,guarded_past_reads=len(opened),guarded_poison_past_reads=len(xopened),no_postdivergence_original_physics_used_as_candidate_truth=True,no_NN_or_simulation_or_new_navigation_result=True,no_actual_asynchronous_worker_admission=True,old_gates_rejects_unchanged=True,whole_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
