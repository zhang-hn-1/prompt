"""Frozen same world-axis evaluator includes all captures, not a policy cursor."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence
from actual_pair_request_fork_v1 import first_actual_request_fork
from local_reserve_execution_classification_v1 import shared_execution_prefix
H=Path(__file__).resolve().parent

def acquisition_speed(velocity):
 return float(np.linalg.norm(np.asarray(velocity,np.float32)[:2]))

def verify(table):
 for name,digest in table.items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name

def arm(name,cfg):
 d=H/('qualified_start_navigation_e1_'+name+'_run_v1');status=json.loads((H/('qualified_start_navigation_e1_'+name+'_status_v1.json')).read_text());assert status['status']=='finished' and status['child_returncode']==0
 seal=json.loads((d/'artifact_manifest_post_close.json').read_text());verify(seal['artifact_sha256']);assert json.loads((d/'candidate_envelope.json').read_text())==dict(arm=name,design=cfg)
 ident=json.loads((d/'identity.json').read_text());assert ident['complete'] and ident['initial_qualification_variant']==name and ident['forward_intent_variant']=='b0' and ident['reset_count']==1 and not ident['internal_reset']
 states=json.loads((d/'states.json').read_text());contacts=json.loads((d/'physics_contacts.json').read_text());physics=dict(np.load(d/'physics.npz'));q=np.load(d/'requests.npz')['requests'];rows=json.loads((d/'policy_records.json').read_text());episode=json.loads((d/'task_episode.json').read_text())['actual_episode'];term=json.loads((d/'termination.json').read_text())
 assert len(q)==len(rows)==ident['actions'] and len(states)==ident['captures']==len(q)+1
 np.testing.assert_array_equal(q,np.asarray([r['request'] for r in rows],np.float32))
 norm=complete_execution_evidence(states,contacts,physics['simulation_time_s'],len(q));start=np.array(episode['start_position_world_m'][:2]);goal=np.array(episode['goal_position_world_m'][:2]);m=assess(states,norm,start,goal)
 assert term['termination']==dict(success='success',collision='collision',timeout='timeout35')[m['outcome']]
 assert len(q)==350 or m['outcome'] in ('success','collision')
 axis=(goal-start)/np.linalg.norm(goal-start);xy=np.array([s['position'][:2] for s in states if s['time_s']<=m['scored_end_s']+1e-8]);cross=np.abs((xy-start)@np.array([-axis[1],axis[0]]));m['cross_axis_all_capture']=dict(rms_m=float(np.sqrt(np.mean(cross**2))),peak_m=float(np.max(cross)),samples=len(cross),scope='0..scored_end')
 count=0;handoff=None
 for r in rows:
  v=acquisition_speed(states[r['frame']]['velocity']);count=count+1 if v<.05 else 0
  a=r['front_view_acquisition']
  assert a['standby_low_count']==count
  if handoff is None and a['handed_off']:
   handoff=r['time_s'];assert count>=5 and (name=='candidate' or handoff>=6.-1e-6)
  assert r['planner_act_called']==(handoff is not None)
  if r['planner_act_called']:assert ident['initial_qualification_time_s']==handoff
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

def main():
 cfg=json.loads((H/'qualified_start_navigation_e1_protocol_v1.json').read_text());verify(cfg['source_sha256']);verify(cfg['input_sha256']);a,ar=arm('b0',cfg);b,br=arm('candidate',cfg);assert ar['episode']==br['episode']
 for k in ('policy_MRO','tracker_MRO','camera_rig','pid_config','inner_config','frozen_source_method'):assert ar['ident'][k]==br['ident'][k],k
 fork=first_actual_request_fork(ar['q'],br['q']);prefix=False
 if fork is not None:
  ma=ar['physics']['simulation_time_s']<=fork*.1+1e-5;mb=br['physics']['simulation_time_s']<=fork*.1+1e-5
  prefix=np.array_equal(ar['q'][:fork],br['q'][:fork]) and all(np.array_equal(ar['physics'][k][ma],br['physics'][k][mb]) for k in ar['physics'] if not k.startswith('raw_')) and shared_execution_prefix(ar['d'],br['d'],fork,len(ar['q']),len(br['q'])) and json.loads((ar['d']/'initial_execution_after_reset.json').read_text())==json.loads((br['d']/'initial_execution_after_reset.json').read_text())
 gates=dict(actual_request_change=fork is not None,prefix_physics_execution_requests_exact=bool(prefix),no_contact=b['raw_contacts_zero'],arrival=b['success'],net_progress=b['along_start_goal_axis_m']>=a['along_start_goal_axis_m'],cross_rms=b['cross_axis_all_capture']['rms_m']<=max(.3,1.25*a['cross_axis_all_capture']['rms_m']),cross_peak=b['cross_axis_all_capture']['peak_m']<=max(.3,1.25*a['cross_axis_all_capture']['peak_m']),latency=b['wall_median_s']<=1.25*a['wall_median_s'] and b['wall_p90_s']<=1.25*a['wall_p90_s'])
 n=min(len(ar['q']),len(br['q']));r=dict(arms={'b0':a,'candidate':b},gates=gates,first_actual_request_fork=fork,different_request_rows=int(np.any(ar['q'][:n]!=br['q'][:n],axis=1).sum()),taskH_candidate_accept=all(gates.values()),classification='conditional_knownpose_TaskH_accept' if all(gates.values()) else 'measured_navigation_reject',not_online_shadow_validation=True,full_causal_claim=False,source_pixel_prefix_not_proven=True,strict_visual_admission=False,estimated_pose_result=None,whole_mission_complete=False)
 p=H/'qualified_start_navigation_e1_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
if __name__=='__main__':main()
