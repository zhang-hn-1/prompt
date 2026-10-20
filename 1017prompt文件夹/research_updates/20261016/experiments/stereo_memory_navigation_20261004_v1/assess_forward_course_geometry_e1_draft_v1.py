"""Frozen same world-axis evaluator includes all captures, not a policy cursor."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
from forward_course_geometry_v1 import select_forward_geometry
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence
from actual_pair_request_fork_v1 import first_actual_request_fork
from local_reserve_execution_classification_v1 import shared_execution_prefix
H=Path(__file__).resolve().parent

from assess_current_tracking_chain_e1_v1 import validate_request_chain

def verify_forward_parent_record(chain,parent,protocol):
 validate_request_chain(chain,parent,protocol)

def acquisition_speed(velocity):
 return float(np.linalg.norm(np.asarray(velocity,np.float32)[:2]))

def verify(table):
 for name,digest in table.items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name

def arm(name,cfg):
 d=H/('forward_course_geometry_e1_'+name+'_run_v1');status=json.loads((H/('forward_course_geometry_e1_'+name+'_status_v1.json')).read_text());assert status['status']=='finished' and status['child_returncode']==0 and not status['timeout'] and not status['forced_kill'] and status['parent_signal'] is None
 seal=json.loads((d/'artifact_manifest_post_close.json').read_text());verify(seal['artifact_sha256']);assert json.loads((d/'candidate_envelope.json').read_text())==dict(arm=name,design=cfg)
 ident=json.loads((d/'identity.json').read_text());assert ident['complete'] and ident['forward_course_geometry_variant']==name and ident['forward_course_geometry_wrapper_count']==1 and ident['current_tracking_wrapper_count']==1 and ident['forward_intent_variant']=='b0' and ident['reset_count']==1 and not ident['internal_reset']
 states=json.loads((d/'states.json').read_text());contacts=json.loads((d/'physics_contacts.json').read_text());physics=dict(np.load(d/'physics.npz'));q=np.load(d/'requests.npz')['requests'];rows=json.loads((d/'policy_records.json').read_text());episode=json.loads((d/'task_episode.json').read_text())['actual_episode'];term=json.loads((d/'termination.json').read_text())
 assert len(q)==len(rows)==ident['actions'] and len(states)==ident['captures']==len(q)+1
 np.testing.assert_array_equal(q,np.asarray([r['request'] for r in rows],np.float32))
 np.testing.assert_array_equal(physics['raw_command_target'][1:],q.reshape(-1,2));assert len(physics['simulation_time_s'])==5*len(q)+1 and len(contacts)==6*len(q)+1
 norm=complete_execution_evidence(states,contacts,physics['simulation_time_s'],len(q));start=np.array(episode['start_position_world_m'][:2]);goal=np.array(episode['goal_position_world_m'][:2]);m=assess(states,norm,start,goal)
 assert term['termination']==dict(success='success',collision='collision',timeout='timeout35')[m['outcome']]
 assert len(q)==350 or m['outcome'] in ('success','collision')
 axis=(goal-start)/np.linalg.norm(goal-start);xy=np.array([s['position'][:2] for s in states if s['time_s']<=m['scored_end_s']+1e-8]);cross=np.abs((xy-start)@np.array([-axis[1],axis[0]]));m['cross_axis_all_capture']=dict(rms_m=float(np.sqrt(np.mean(cross**2))),peak_m=float(np.max(cross)),samples=len(cross),scope='0..scored_end')
 count=0;handoff=None;history=[]
 for r in rows:
  v=acquisition_speed(states[r['frame']]['velocity']);count=count+1 if v<.05 else 0
  a=r['front_view_acquisition']
  assert a['standby_low_count']==count
  if handoff is None and a['handed_off']:
   handoff=r['time_s'];assert count>=5 and handoff>=6.-1e-6
  assert r['planner_act_called']==(handoff is not None)
  if r['planner_act_called']:
   prop=np.load(d/'history'/f"frame_{r['frame']:03d}.npz")['proprio_history'][-1]
   np.testing.assert_array_equal(prop[:2],np.asarray(states[r['frame']]['velocity'][:2],np.float32))
   history.append((r['time_s'],prop[:2].copy(),float(prop[12])));history=history[-5:]
   decision=r['forward_course_geometry'];chain=r.get('current_tracking_chain',{})
   if decision.get('valid'):
    parent=float(chain['geometric_pure_pursuit_curvature_1_m'])
    delta=np.array([chain['lookahead_x_m'],chain['lookahead_y_m']])-np.asarray(states[r['frame']]['position'][:2])
    yaw=float(chain['root_heading_rad']);den=float(delta@delta)
    expected_parent=0. if den<=1e-9 else 2*(-np.sin(yaw)*delta[0]+np.cos(yaw)*delta[1])/den
    assert abs(parent-expected_parent)<1e-8
    ref=r['current_reference_waypoints_after_act'][chain['projection_index']]
    if name=='candidate' and abs(ref['main_command_rad_s'])>1e-12:
     selected,expected=select_forward_geometry(parent,delta,now=r['time_s'],times=[v[0] for v in history],velocity=[v[1] for v in history],main_velocity=[v[2] for v in history],active=r['tracker_state_after_act']['active_direction'],pending=r['tracker_state_after_act']['pending_direction'],braking='tracking',reference_direction=ref['drive_direction'])
     assert decision['used']==expected['used']
     assert abs(selected-chain['requested_curvature_1_m'])<1e-8
     if expected['used']:assert abs(expected['course_heading_rad']-decision['course_heading_rad'])<1e-8
    else:
     assert not decision['used'];assert abs(parent-chain['requested_curvature_1_m'])<1e-8
    if chain.get('source')=='tracking' and chain.get('valid'):validate_request_chain(chain,np.asarray(r['request'],np.float32),protocol)
   else:assert not decision['used']

  assert r['known_GT_pose'] and not r['native_pose_used']
 videos={}
 for n in ('stereo_view','third_person'):
  v=cv2.VideoCapture(str(d/(n+'.mp4')));fps=v.get(cv2.CAP_PROP_FPS);nframes=0
  while v.read()[0]:nframes+=1
  v.release();assert fps==10. and nframes==len(states);videos[n]=dict(frames=nframes,fps=fps)
 clocks=json.loads((d/'camera_clocks.json').read_text());assert len(clocks)==len(states)
 assert all(abs(c['eyes'][e]-c['raw_time_s'])<=1e-7 for c in clocks for e in ('left','right'))
 wall=np.array([r['end_to_end_wall_s'] for r in rows]);m.update(actual_handoff_time_s=handoff,requests=len(q),captures=len(states),physics_samples=len(physics['simulation_time_s']),contact_samples=len(contacts),raw_contacts_zero=not any(c['contact'] for c in contacts),sealed_items=len(seal['artifact_sha256']),videos=videos,wall_median_s=float(np.median(wall)),wall_p90_s=float(np.percentile(wall,90)),source_gaps=observation_gap_diagnostics(json.loads((d/'source_rows.json').read_text()),rows))
 return m,dict(d=d,q=q,physics=physics,states=states,ident=ident,episode=episode)

def intervention_evidence(first_fork,brows,crows):
 frames=[r['frame'] for r in crows if r.get('forward_course_geometry',{}).get('used',False)]
 isolated=False
 if first_fork is not None and first_fork in frames:
  a=brows[first_fork];b=crows[first_fork];ca=a.get('current_tracking_chain',{});cb=b.get('current_tracking_chain',{})
  isolated=(a['current_reference_waypoints_after_act']==b['current_reference_waypoints_after_act'] and a['tracker_state_after_act']==b['tracker_state_after_act'] and all(ca.get(k)==cb.get(k) for k in ('root_heading_rad','lookahead_x_m','lookahead_y_m','geometric_pure_pursuit_curvature_1_m','projection_index')) and abs(b['forward_course_geometry']['selected_curvature']-b['forward_course_geometry']['parent_curvature'])>1e-12)
 return dict(count=len(frames),first_geometry_event=None if not frames else min(frames),first_pair_fork=first_fork,consumer_fork_isolated=bool(isolated))

def main():
 cfg=json.loads((H/'forward_course_geometry_e1_protocol_v1.json').read_text());verify(cfg['source_sha256']);verify(cfg['input_sha256']);a,ar=arm('b0',cfg);b,br=arm('candidate',cfg);assert ar['episode']==br['episode']
 for k in ('policy_MRO','tracker_MRO','camera_rig','pid_config','inner_config','frozen_source_method'):assert ar['ident'][k]==br['ident'][k],k
 fork=first_actual_request_fork(ar['q'],br['q']);prefix=False
 if fork is not None:
  ma=ar['physics']['simulation_time_s']<=fork*.1+1e-5;mb=br['physics']['simulation_time_s']<=fork*.1+1e-5
  prefix=np.array_equal(ar['q'][:fork],br['q'][:fork]) and all(np.array_equal(ar['physics'][k][ma],br['physics'][k][mb]) for k in ar['physics'] if not k.startswith('raw_')) and shared_execution_prefix(ar['d'],br['d'],fork,len(ar['q']),len(br['q'])) and json.loads((ar['d']/'initial_execution_after_reset.json').read_text())==json.loads((br['d']/'initial_execution_after_reset.json').read_text())
 intervention=intervention_evidence(fork,json.loads((ar['d']/'policy_records.json').read_text()),json.loads((br['d']/'policy_records.json').read_text()))
 gates=dict(actual_request_change=fork is not None and intervention['count']>0,consumer_fork_isolated=intervention['consumer_fork_isolated'],prefix_physics_execution_requests_exact=bool(prefix),no_contact=b['raw_contacts_zero'],arrival=b['success'],net_progress=b['along_start_goal_axis_m']>=a['along_start_goal_axis_m'],cross_rms=b['cross_axis_all_capture']['rms_m']<=max(.3,1.25*a['cross_axis_all_capture']['rms_m']),cross_peak=b['cross_axis_all_capture']['peak_m']<=max(.3,1.25*a['cross_axis_all_capture']['peak_m']),latency=b['wall_median_s']<=1.25*a['wall_median_s'] and b['wall_p90_s']<=1.25*a['wall_p90_s'])
 n=min(len(ar['q']),len(br['q']));r=dict(intervention=intervention,arms={'b0':a,'candidate':b},gates=gates,first_actual_request_fork=fork,different_request_rows=int(np.any(ar['q'][:n]!=br['q'][:n],axis=1).sum()),taskH_candidate_accept=all(gates.values()),classification='unassessed_event_or_interface' if not (gates['actual_request_change'] and gates['consumer_fork_isolated'] and gates['prefix_physics_execution_requests_exact']) else ('conditional_knownpose_TaskH_accept' if all(gates.values()) else 'measured_navigation_reject'),not_online_shadow_validation=True,full_causal_claim=False,source_pixel_prefix_not_proven=True,nonzeroaux_model_admitted=False,current_course_geometric_consumer_is_empirical=True,strict_visual_admission=False,estimated_pose_result=None,whole_mission_complete=False)
 p=H/'forward_course_geometry_e1_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
if __name__=='__main__':main()
