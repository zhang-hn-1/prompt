from pathlib import Path
import json,hashlib
import numpy as np
import cv2
from pacing_reference_metrics_v1 import measure,full_contact_evidence
from local_reserve_execution_classification_v1 import classify,shared_execution_prefix
H=Path(__file__).resolve().parent

def verify(mapping):
 for f,d in mapping.items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==d,f
 return len(mapping)

def main():
 cfg=json.loads((H/'measured_speed_fixed_reference_protocol_v1.json').read_text());verify(cfg['source_sha256']);verify(cfg['input_sha256'])
 old_cfg=json.loads((H/'fixed_reference_pacing_knownpose_protocol_v3.json').read_text());verify(old_cfg['source_sha256']);verify(old_cfg['input_sha256'])
 ref=json.loads((H/'straight_pacing_fixed_reference_v1.json').read_text());path=np.array([[w['x_m'],w['y_m']] for w in ref['waypoints']]);arms={};raw={}
 for arm in ('b0','paced'):
  D=H/('measured_speed_fixed_reference_'+arm+'_run_v1');status=json.loads((H/('measured_speed_fixed_reference_'+arm+'_status_v1.json')).read_text())
  manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());sealed=verify(manifest['artifact_sha256']);identity=json.loads((D/'identity.json').read_text())
  assert json.loads((D/'candidate_envelope.json').read_text())==dict(arm=arm,design=cfg)
  rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());actions=np.load(D/'requests.npz')['requests'];physics=dict(np.load(D/'physics.npz'));contacts=json.loads((D/'physics_contacts.json').read_text())
  assert len(actions)==len(rows) and len(states)==len(actions)+1
  closure=classify(status,True,identity,states,contacts,physics['simulation_time_s'],len(actions))
  if closure['classification'] not in ('complete_35','measured_collision_reject'):raise ValueError('unassessed closure: '+str(closure))
  assert np.array_equal(actions,np.asarray([r['request'] for r in rows],np.float32))
  assert identity['reference_sha256']==hashlib.sha256((H/'straight_pacing_fixed_reference_v1.json').read_bytes()).hexdigest()
  for r in rows:
   if r.get('planner_act_called'):assert r['installed_reference_xy_after_act']==path.tolist() and r['local_reference_version']==1
  videos={}
  for n in ['stereo_view','third_person']:
   c=cv2.VideoCapture(str(D/(n+'.mp4')));fps=c.get(cv2.CAP_PROP_FPS);count=0
   while c.read()[0]:count+=1
   c.release();assert count==len(states) and fps==10.;videos[n]=dict(decoded_frames=count,fps=fps)
  installs=[r for r in rows if r.get('local_plan_created')];assert len(installs)<=1
  if closure['complete_35']:assert len(installs)==1
  install_s=float(installs[0]['reference_installed_time_s']) if installs else None
  if installs:assert abs(install_s-float(installs[0]['time_s']))<1e-8
  scored_states=[x for x in states if x['time_s']<=closure['scored_end_s']+1e-6]
  m=measure(scored_states,path,install_s) if install_s is not None and any(x['time_s']>=install_s-1e-5 for x in scored_states) else dict(endpoint_final_m=float('inf'),first_endpoint_passage_from_install_s=None,terminal_stop=dict(last_count=0),cross_track_RMS_m=float('inf'),cross_track_peak_m=float('inf'))
  m.update(execution_closure=closure,actual_reference_install_s=install_s,available_contacts_zero=full_contact_evidence(contacts,len(actions),float(states[-1]['time_s'])),sealed_items=sealed,videos=videos,policy_identity=identity['policy_MRO'],all_fixed_reference=True)
  arms[arm]=m;raw[arm]=dict(states=states,rows=rows,actions=actions,physics=physics,initial=json.loads((D/'initial_execution_after_reset.json').read_text()))
 events=np.flatnonzero([r.get('measured_speed_control_applied',False) for r in raw['paced']['rows']]);fork=None if not len(events) else int(events[0]);prefix_match=False;phys_match=False;exec_match=False
 if fork is not None:
  prefix_match=np.array_equal(raw['b0']['actions'][:fork],raw['paced']['actions'][:fork])
  # Physics at/before the actual intervention boundary only; raw world clock differs across fresh simulation processes.
  p,q=raw['b0']['physics'],raw['paced']['physics'];mask=np.asarray(p['simulation_time_s'])<=fork*.1+1e-5;mask2=np.asarray(q['simulation_time_s'])<=fork*.1+1e-5
  keys=[k for k in p if k not in ('raw_simulation_time_s','raw_physics_step_index')];phys_match=all(np.array_equal(np.asarray(p[k])[mask],np.asarray(q[k])[mask2]) for k in keys)
  Db=H/'measured_speed_fixed_reference_b0_run_v1';Dp=H/'measured_speed_fixed_reference_paced_run_v1'
  exec_match=shared_execution_prefix(Db,Dp,fork,len(raw['b0']['actions']),len(raw['paced']['actions']))
  assert np.array_equal(raw['b0']['actions'][fork,1::2],raw['paced']['actions'][fork,1::2]) if prefix_match and phys_match and exec_match else True
 b,c=arms['b0'],arms['paced'];bt,ct=b['first_endpoint_passage_from_install_s'],c['first_endpoint_passage_from_install_s']
 gates=dict(complete_two_sequences=b['execution_closure']['complete_35'] and c['execution_closure']['complete_35'],available_contacts_zero=b['available_contacts_zero'] and c['available_contacts_zero'],prefix_match=prefix_match and phys_match and exec_match and raw['b0']['initial']==raw['paced']['initial'] and b['actual_reference_install_s'] is not None and c['actual_reference_install_s'] is not None and abs(b['actual_reference_install_s']-c['actual_reference_install_s'])<1e-8,request_changed=fork is not None and not np.array_equal(raw['b0']['actions'],raw['paced']['actions']),final_endpoint=c['endpoint_final_m']<=.2,terminal_stop=c['terminal_stop']['last_count']>=5,cross_RMS=c['cross_track_RMS_m']<=.1,cross_peak=c['cross_track_peak_m']<=.2,first_endpoint_passage_20percent_faster=bt is not None and ct is not None and ct<=.8*bt)
 from measured_speed_consumer_v1 import FEEDFORWARD,KP,TARGET,RESERVE
 for row in raw['paced']['rows']:
  if not row.get('planner_act_called'):continue
  before=np.array(row['measured_speed_before_request'],np.float32);issued=np.array(row['request'],np.float32)
  assert np.array_equal(before[1::2],issued[1::2])
  assert row['measured_speed_control_applied']==(not np.array_equal(before,issued))
  if row['measured_speed_control_eligible']:
   assert row['measured_speed_local_endpoint_m']>=RESERVE and row['measured_speed_global_distance_m']>=RESERVE
   speed=np.linalg.norm(np.array(raw['paced']['states'][row['frame']]['velocity'],np.float32)[:2])
   assert abs(speed-row['measured_speed_worldXY_m_s'])<1e-6
   expected=np.float32(np.clip(FEEDFORWARD+KP*(TARGET-row['measured_speed_worldXY_m_s']),.5,1.))
   assert np.all(issued[::2]==expected)
  else:assert np.array_equal(before,issued)
 result=dict(arms=arms,gates=gates,classification='accept' if all(gates.values()) else 'measured_control_reject',scientific_rejection=not all(gates.values()),local_candidate_accept=all(gates.values()),first_intervention_frame=fork,intervention_count=len(events),prefix_details=dict(actions_match=prefix_match,all_nonraw_physics_match=phys_match,execution_states_match=exec_match),no_formal_visual_navigation_admission=True,formal_navigation_result=None,whole_mission_complete=False)
 out=H/'measured_speed_fixed_reference_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':
 try:main()
 except (AssertionError,ValueError,KeyError,OSError) as e:
  statuses={}
  for arm in ('b0','paced'):
   f=H/('measured_speed_fixed_reference_'+arm+'_status_v1.json')
   statuses[arm]=json.loads(f.read_text()) if f.exists() else None
  execution_failure=any(s is None or s.get('status')!='finished' or s.get('child_returncode')!=0 for s in statuses.values())
  result=dict(classification='unassessed_execution' if execution_failure else 'unassessed_evidence',reason=repr(e),statuses=statuses,local_candidate_accept=False,scientific_rejection=False,whole_mission_complete=False)
  out=H/'measured_speed_fixed_reference_unassessed_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)

