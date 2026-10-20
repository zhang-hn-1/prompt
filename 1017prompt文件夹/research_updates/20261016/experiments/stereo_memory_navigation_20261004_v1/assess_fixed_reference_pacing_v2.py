from pathlib import Path
import json,hashlib
import numpy as np
import cv2
from pacing_reference_metrics_v1 import measure,full_contact_evidence
H=Path(__file__).resolve().parent

def verify(mapping):
 for f,d in mapping.items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==d,f
 return len(mapping)

def main():
 cfg=json.loads((H/'fixed_reference_pacing_protocol_v2.json').read_text());verify(cfg['source_sha256']);verify(cfg['input_sha256'])
 ref=json.loads((H/'straight_pacing_fixed_reference_v1.json').read_text());path=np.array([[w['x_m'],w['y_m']] for w in ref['waypoints']]);arms={};raw={}
 for arm in ('b0','paced'):
  D=H/('fixed_reference_pacing_'+arm+'_run_v2');status=json.loads((H/('fixed_reference_pacing_'+arm+'_status_v2.json')).read_text());assert status['status']=='finished' and status['child_returncode']==0
  manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());sealed=verify(manifest['artifact_sha256']);identity=json.loads((D/'identity.json').read_text());assert identity['complete'] and identity['actions']==350 and identity['captures']==351
  rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());actions=np.load(D/'requests.npz')['requests'];physics=dict(np.load(D/'physics.npz'));contacts=json.loads((D/'physics_contacts.json').read_text())
  assert len(actions)==len(rows)==350 and len(states)==351
  assert np.array_equal(actions,np.asarray([r['request'] for r in rows],np.float32))
  assert identity['reference_sha256']==hashlib.sha256((H/'straight_pacing_fixed_reference_v1.json').read_bytes()).hexdigest()
  for r in rows:
   if r.get('planner_act_called'):assert r['installed_reference_xy_after_act']==path.tolist() and r['local_reference_version']==1
  videos={}
  for n in ['stereo_view','third_person']:
   c=cv2.VideoCapture(str(D/(n+'.mp4')));fps=c.get(cv2.CAP_PROP_FPS);count=0
   while c.read()[0]:count+=1
   c.release();assert count==351 and fps==10.;videos[n]=dict(decoded_frames=count,fps=fps)
  installs=[r for r in rows if r.get('local_plan_created')];assert len(installs)==1
  install_s=float(installs[0]['reference_installed_time_s']);assert abs(install_s-float(installs[0]['time_s']))<1e-8
  m=measure(states,path,install_s);m.update(actual_reference_install_s=install_s,available_contacts_zero=full_contact_evidence(contacts,len(actions),float(states[-1]['time_s'])),sealed_items=sealed,videos=videos,policy_identity=identity['policy_MRO'],all_fixed_reference=True)
  arms[arm]=m;raw[arm]=dict(states=states,rows=rows,actions=actions,physics=physics,initial=json.loads((D/'initial_execution_after_reset.json').read_text()))
 events=np.flatnonzero([r.get('straight_pacing_applied',False) for r in raw['paced']['rows']]);fork=None if not len(events) else int(events[0]);prefix_match=False;phys_match=False;exec_match=False
 if fork is not None:
  prefix_match=np.array_equal(raw['b0']['actions'][:fork],raw['paced']['actions'][:fork])
  # Physics at/before the actual intervention boundary only; raw world clock differs across fresh simulation processes.
  p,q=raw['b0']['physics'],raw['paced']['physics'];mask=np.asarray(p['simulation_time_s'])<=fork*.1+1e-5;mask2=np.asarray(q['simulation_time_s'])<=fork*.1+1e-5
  keys=[k for k in p if k not in ('raw_simulation_time_s','raw_physics_step_index')];phys_match=all(np.array_equal(np.asarray(p[k])[mask],np.asarray(q[k])[mask2]) for k in keys)
  Db=H/'fixed_reference_pacing_b0_run_v2';Dp=H/'fixed_reference_pacing_paced_run_v2'
  exec_match=all(json.loads((Db/'execution'/f'frame_{i:03d}.json').read_text())==json.loads((Dp/'execution'/f'frame_{i:03d}.json').read_text()) for i in range(fork+1))
  assert np.array_equal(raw['b0']['actions'][fork,1::2],raw['paced']['actions'][fork,1::2]) if prefix_match and phys_match else True
 b,c=arms['b0'],arms['paced'];bt,ct=b['first_endpoint_passage_from_install_s'],c['first_endpoint_passage_from_install_s']
 gates=dict(complete_two_sequences=True,available_contacts_zero=b['available_contacts_zero'] and c['available_contacts_zero'],prefix_match=prefix_match and phys_match and exec_match and raw['b0']['initial']==raw['paced']['initial'] and abs(b['actual_reference_install_s']-c['actual_reference_install_s'])<1e-8,request_changed=fork is not None and not np.array_equal(raw['b0']['actions'],raw['paced']['actions']),final_endpoint=c['endpoint_final_m']<=.2,terminal_stop=c['terminal_stop']['last_count']>=5,cross_RMS=c['cross_track_RMS_m']<=.1,cross_peak=c['cross_track_peak_m']<=.2,first_endpoint_passage_20percent_faster=bt is not None and ct is not None and ct<=.8*bt)
 result=dict(arms=arms,gates=gates,local_candidate_accept=all(gates.values()),first_intervention_frame=fork,intervention_count=len(events),prefix_details=dict(actions_match=prefix_match,all_nonraw_physics_match=phys_match,execution_states_match=exec_match),no_formal_visual_navigation_admission=True,formal_navigation_result=None,whole_mission_complete=False)
 out=H/'fixed_reference_pacing_assessment_v2.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
