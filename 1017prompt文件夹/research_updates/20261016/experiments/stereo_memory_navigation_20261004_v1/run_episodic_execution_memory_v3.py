"""One bounded offline retrieval library/prediction run, then separately sealed teacher phase."""
from pathlib import Path
import json,hashlib,datetime,argparse,math
import numpy as np
from episodic_execution_memory_v2 import features,mint_hold_proposal,Memory
H=Path(__file__).resolve().parent
TRAIN=('sgbm_baseline35_e0_coverage_run_v1','sgbm_baseline35_e1_coverage_run_v1','sgbm_baseline35_e2_run_v1','native_pose_baseline35_e0_run_v1')
TEST=('native_pose_baseline35_e1_coverage_run_v1','native_pose_baseline35_e2_coverage_run_v1')
OUT=H/'episodic_execution_memory_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,x):
 assert not p.exists(),str(p)
 p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def yaw(q):
 q=np.asarray(q,float);q=q/np.linalg.norm(q);w,x,y,z=q
 return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def pair(row):
 a=np.asarray(row['request'],float).reshape(5,2)
 if not np.array_equal(a,np.tile(a[0],(5,1))):raise ValueError('actual block must repeat one pair')
 return a[0]
def feature_at(root,f,rows):
 z=np.load(root/'history'/f'frame_{f:03d}.npz');now=float(rows[f]['time_s']);a=z['action_history'].reshape(5,5,2)
 if not np.array_equal(a,np.repeat(a[:,:1,:],5,axis=1)):raise ValueError('past block pair repetitions required')
 packet=dict(times=np.array([rows[i]['time_s'] for i in range(f-4,f+1)],float),proprio=z['proprio_history'],past_actions=a[:,0],yaw_current=yaw(z['quaternion_world_wxyz_history'][-1]),privilege='simulator_state_diagnostic')
 return features(packet,now),mint_hold_proposal(*pair(rows[f]),now),packet

def honored(rows,f,commands):
 if f+5>len(rows):return False
 actual=np.stack([pair(rows[i]) for i in range(f,f+5)])
 d=np.abs(actual-commands)
 return bool((d[:,0]<=.01+1e-12).all() and (d[:,1]<=math.radians(2)+1e-12).all())
def target_at(states,f):
 if f+5>=len(states):return None
 anchor=states[f];t=float(anchor['time_s']);q=yaw(anchor['quaternion']);c,s=math.cos(q),math.sin(q);rotation=np.array([[c,s],[-s,c]])
 result=[]
 for i in range(1,6):
  state=states[f+i]
  if abs(float(state['time_s'])-t-i*.1)>1e-6:raise ValueError('teacher capture clocks differ from horizon')
  xy=(np.asarray(state['position'])[:2]-np.asarray(anchor['position'])[:2])@rotation.T;angle=(yaw(state['quaternion'])-q+math.pi)%(2*math.pi)-math.pi;result.append([*xy,angle])
 return np.array(result)
def precontact(root,end):
 records=json.loads((root/'physics_contacts.json').read_text());selected=[r for r in records if r['simulation_time_s']<=end+1e-6]
 return bool(selected) and all(r['available'] and not r['contact'] for r in selected) and max(r['simulation_time_s'] for r in selected)>=end-1e-6

def cv(packet):
 p=packet['proprio'][-1];angle=packet['yaw_current'];c,s=math.cos(angle),math.sin(angle);v=p[:2]@np.array([[c,s],[-s,c]]).T;t=np.arange(1,6)*.1
 return np.c_[t[:,None]*v,t*p[5]]
def four_point(proposal):
 from execution_domain_four_point_source_v2 import supported_curvature
 from supported_forward_primitive_source import _LEVEL
 main,aux=proposal.commands[0]
 # Exact issued domain; native float32 storage gets only representation tolerance.
 if abs(main-.5)<1e-6:
  exact=next((x for x in [0.,-math.pi/6,math.pi/6] if abs(aux-x)<1e-6),None)
  if exact is None:return None
  k=supported_curvature(.5,exact);speed=float(_LEVEL['speed_m_s'])
 elif abs(main+.3)<1e-6 and abs(aux)<1e-6:
  k=supported_curvature(-.5,0.);speed=-float(json.loads((H/'reverse_straight_fitted_point_v1.json').read_text())['measured']['actual_speed_m_s'])
 else:return None
 t=np.arange(1,6)*.1;a=k*speed*t
 x=speed*t if abs(k)<1e-12 else np.sin(a)/k;y=np.zeros(5) if abs(k)<1e-12 else (1-np.cos(a))/k
 return np.c_[x,y,a]
def validate_identity():
 protocol=json.loads((H/'episodic_execution_memory_protocol_v3.json').read_text())
 for p,s in protocol['identity_sha256'].items():assert sha(p)==s,p
 for run in TRAIN+TEST:
  root=H/run;seal=json.loads((root/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
  for p,s in protocol['input_sha256'].items():
   if root in Path(p).parents:assert seal[p]==s and sha(p)==s,p
 return protocol

def predict():
 protocol=validate_identity();assert not OUT.exists();OUT.mkdir()
 entries=[];counts={};train_arrays=[]
 for episode in TRAIN:
  root=H/episode;rows=json.loads((root/'policy_records.json').read_text());states=json.loads((root/'states.json').read_text());counts[episode]={'considered':0,'library':0}
  for f in range(5,341,5):
   counts[episode]['considered']+=1;feat,proposal,packet=feature_at(root,f,rows);target=target_at(states,f)
   if target is None or not honored(rows,f,proposal.commands) or not precontact(root,proposal.created_at+.5):continue
   entries.append(dict(episode=episode,frame=f,feature=feat,program=proposal.commands,target=target));counts[episode]['library']+=1
 library=Memory(entries,allowed_episodes=set(TRAIN),forbidden_episodes=set(TEST))
 np.savez_compressed(OUT/'library.npz',feature=np.array([e['feature'] for e in entries]).reshape(-1,110),program=np.array([e['program'] for e in entries]).reshape(-1,5,2),target=np.array([e['target'] for e in entries]).reshape(-1,5,3),episode=np.array([e['episode'] for e in entries]),frame=np.array([e['frame'] for e in entries]))
 predictions=[]
 for episode in TEST:
  root=H/episode;rows=json.loads((root/'policy_records.json').read_text())
  for f in range(60,201,10):
   rec=dict(episode=episode,frame=f,privilege='simulator_current_state_diagnostic_not_pure_RGB',offline_replay_not_online=True)
   if f>=len(rows) or not (root/'history'/f'frame_{f:03d}.npz').exists():rec.update(status='unavailable_anchor',trajectory=None);predictions.append(rec);continue
   feat,proposal,packet=feature_at(root,f,rows);got=library.query(feat,proposal,now=proposal.created_at);base4=four_point(proposal)
   rec.update(feature=feat.tolist(),proposal=dict(commands=proposal.commands.tolist(),created_at=proposal.created_at,producer=proposal.producer),constant_velocity=cv(packet).tolist(),four_point=None if base4 is None else base4.tolist(),**{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in got.items()});predictions.append(rec)
 dump(OUT/'predictions.json',predictions)
 dump(OUT/'prediction_seal.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),protocol_sha256=sha(H/'episodic_execution_memory_protocol_v3.json'),design_sha256=sha(H/'episodic_execution_memory_design_v3.json'),library_sha256=sha(OUT/'library.npz'),prediction_sha256=sha(OUT/'predictions.json'),counts=counts,heldout_future_states_opened=False,queries=len(predictions),prediction_run_complete=True))
 print('SEALED',len(entries),'library entries;',len(predictions),'test anchors')

def verify_prediction_seal(seal,predictions,protocol_sha,design_sha):
 expected={(e,f) for e in TEST for f in range(60,201,10)}
 if seal.get('protocol_sha256')!=protocol_sha or seal.get('design_sha256')!=design_sha:raise ValueError('prediction seal protocol/design mismatch')
 if seal.get('prediction_run_complete') is not True or seal.get('heldout_future_states_opened') is not False or seal.get('queries')!=30:raise ValueError('incomplete prediction seal')
 actual=[(p.get('episode'),p.get('frame')) for p in predictions]
 if len(actual)!=30 or len(set(actual))!=30 or set(actual)!=expected:raise ValueError('fixed unique anchor set mismatch')

def score():
 validate_identity();seal=json.loads((OUT/'prediction_seal.json').read_text());assert sha(OUT/'predictions.json')==seal['prediction_sha256'] and sha(OUT/'library.npz')==seal['library_sha256'];assert not (OUT/'assessment.json').exists()
 predictions=json.loads((OUT/'predictions.json').read_text());verify_prediction_seal(seal,predictions,sha(H/'episodic_execution_memory_protocol_v3.json'),sha(H/'episodic_execution_memory_design_v3.json'))
 rows=[];summary={};total=0;design=json.loads((H/'episodic_execution_memory_design_v3.json').read_text());g=design['gates']
 for episode in TEST:
  root=H/episode;actual=json.loads((root/'policy_records.json').read_text());states=json.loads((root/'states.json').read_text());errors=[];bases=[];yaws=[];four=[];reasons={}
  for pred in predictions:
   if pred['episode']!=episode:continue
   f=pred['frame'];r=dict(episode=episode,frame=f,prediction_status=pred['status']);target=target_at(states,f) if f<len(states) else None
   reason=pred['status'] if pred['status']!='supported' else None
   if reason is None and (target is None or not precontact(root,pred['proposal']['created_at']+.5)):reason='unavailable_or_postcontact_teacher'
   if reason is None and not honored(actual,f,np.array(pred['proposal']['commands'])):reason='realized_program_does_not_honor_proposal'
   if reason is not None:r['score_status']=reason;reasons[reason]=reasons.get(reason,0)+1;rows.append(r);continue
   estimate=np.array(pred['trajectory']);base=np.array(pred['constant_velocity']);e=np.linalg.norm(estimate[:,:2]-target[:,:2],axis=1);b=np.linalg.norm(base[:,:2]-target[:,:2],axis=1);angle=np.abs((estimate[:,2]-target[:,2]+math.pi)%(2*math.pi)-math.pi)
   errors.extend(e);bases.extend(b);yaws.extend(angle);total+=1;r.update(score_status='scored',target=target.tolist(),XY_error=e.tolist(),CV_error=b.tolist(),yaw_error_rad=angle.tolist())
   if pred['four_point'] is not None:
    e4=np.linalg.norm(np.array(pred['four_point'])[:,:2]-target[:,:2],axis=1);four.extend(e4);r['four_point_error']=e4.tolist()
   rows.append(r)
  n=len(errors)//5;summary[episode]=dict(fixed_anchors=15,scored_supported_honored=n,reasons=reasons,XY_RMSE_m=float(np.sqrt(np.mean(np.square(errors)))) if n else None,CV_RMSE_m=float(np.sqrt(np.mean(np.square(bases)))) if n else None,XY_p90_m=float(np.quantile(errors,.9)) if n else None,yaw_p90_deg=float(np.rad2deg(np.quantile(yaws,.9))) if n else None,four_point_scored_steps=len(four),four_point_RMSE_m=float(np.sqrt(np.mean(np.square(four)))) if four else None)
 coverage=total>=g['supported_and_proposal_honored_total_min'] and all(v['scored_supported_honored']>=g['supported_and_proposal_honored_each_episode_min'] for v in summary.values())
 passing=coverage and all(v['XY_RMSE_m']<=g['XY_RMSE_each_episode_max_m'] and v['XY_RMSE_m']<=g['XY_RMSE_each_episode_vs_CV_max_ratio']*v['CV_RMSE_m'] and v['XY_p90_m']<=g['XY_p90_each_episode_max_m'] and v['yaw_p90_deg']<=g['yaw_p90_each_episode_max_deg'] for v in summary.values())
 dump(OUT/'assessment.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),prediction_seal_sha256=sha(OUT/'prediction_seal.json'),summary=summary,scored_supported_honored=total,fixed_anchors=30,coverage_pass=coverage,error_gate_pass=passing,classification='coverage_unassessed' if not coverage else ('offline_motion_prediction_pass' if passing else 'fixed_retrieval_reject'),rows=rows,no_runtime_admission=True,new_physics_runs=0,new_CAD=0,new_network_fits=0,whole_mission_complete=False));print(summary)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('phase',choices=['predict','score']);arg=a.parse_args();predict() if arg.phase=='predict' else score()
