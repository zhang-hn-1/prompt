"""Bounded new-sequence calibration, frozen legacy gates, conditional sealed fresh eval."""
from pathlib import Path
import json,hashlib,time,os,sys,subprocess,importlib.util,traceback
import numpy as np
import torch
from confidence_gate import fit_gate,state_features,scale_targets,GatedPredictor
from training.acceleration_residual import AccelerationResidualPredictor
from training.motion_residual import CVResidualPredictor,build_window
from training.ball_wm_dataset import BallWMDataset,_load_episode,_load_image,_normalize_image,_proprio17
from training.joint_jepa import JointModelConfig,build_joint_model
from scripts.run_wm_motion_residual import tags,evaluate
R=Path(__file__).resolve().parent;read=lambda p:json.loads(Path(p).read_text())
p=read(R/'protocol.json');ROOT=Path(p['root']);A=Path(p['frozen_acceleration_run']);G=Path(p['parent_gate_run']);O=Path(p['frozen_rate_comparator']);oldp=read(A/'protocol.json')
def dump(name,v):(R/name).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
resume_amendment=read(R/'calibration_coverage_amendment.json')
for f,h in resume_amendment['preserved_original_artifacts'].items():assert sha(R/f)==h,f
assert not (R/'gate.pt').exists(), 'Amendment must predate fitting'
assert not (R/'resume_status.json').exists(),'Refuse overwrite'
status={'status':'running','started_epoch':time.time(),'phase':'integrity'};dump('resume_status.json',status)
def phase(name):status['phase']=name;dump('resume_status.json',status);print(name,flush=True)
env=os.environ.copy();env['PYTHONPATH']='/home/an/.cache/ball_wm_visibility_numba_0612:'+str(ROOT/'research/ball_wm')+':'+str(ROOT/'research/ball_wm/scripts');env['PYTHONUNBUFFERED']='1';env['PYTHONDONTWRITEBYTECODE']='1';env['NUMBA_CACHE_DIR']='/tmp/wm_independent_calibration_numba_20260926';env['MPLCONFIGDIR']='/tmp/wm_independent_calibration_mpl_20260926'
spec=importlib.util.spec_from_file_location('original_diverse_audit',ROOT/'research/ball_wm/runs/wm_aux30_diverse_20260922_v1/audit_diverse.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
collection_records=[]
def collect(jobs,group):
 from audit_dataset import audit_episode_report
 from calibration_eligibility import check_eligibility
 for job in jobs:
  name=job['id'];ep=Path(job['output'])/'ep0000';log=R/(name+'.log')
  assert sha(job['command_sequence'])==job['command_sequence_sha256'];phase('collect_'+group+'_'+name)
  resumed=ep.exists()
  if resumed:
   assert group=='calibration' and name in resume_amendment['preserved_raw_states_sha256']
   assert sha(ep/'states.npz')==resume_amendment['preserved_raw_states_sha256'][name]
   record=read(R/(name+'_execution.json'));assert record['finalized'] and not ep.with_suffix('.partial').exists()
  else:
   assert not ep.with_suffix('.partial').exists() and not log.exists(),name
   record={'id':name,'group':group,'command':job['command'],'started_epoch':time.time(),'sequence_sha256':sha(job['command_sequence'])}
   dump(name+'_execution.json',record)
   with log.open('w') as out:result=subprocess.run(job['command'],cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900)
   record.update(returncode=result.returncode,wall_seconds=time.time()-record['started_epoch'],finalized=ep.is_dir() and (ep/'states.npz').exists() and not ep.with_suffix('.partial').exists());dump(name+'_execution.json',record)
   assert result.returncode==0 and record['finalized'],f'Collection failed: {name}'
  report=audit_episode_report(ep);dump('resume_schema_'+name+'.json',report)
  eligibility=check_eligibility(name,group,sha(job['command_sequence']),report['errors'])
  assert record['returncode']==(1 if eligibility['coverage_exception'] else 0)
  qc=audit.inspect_episode(job)
  with np.load(ep/'states.npz') as z:qc['first_measured_world_position']=z['pos'][0].tolist();qc['first_measured_quaternion']=z['quat'][0].tolist();qc['first_position_note']='First post-step sample, not exact reset position.'
  dump('resume_qc_'+name+'.json',qc);assert qc['passed'] and all(qc['checks'].values()),f'Data QC failed: {name}'
  record.update(schema_audit_returncode=0 if eligibility['formal_schema_passed'] else 1,quality_passed=qc['passed'],resumed_existing_episode=resumed,**eligibility)
  collection_records.append(record);dump('resume_collection_summary.json',collection_records)
  print('COLLECTED',name,'formal_schema_passed',eligibility['formal_schema_passed'],'causal_QC',qc['passed'],flush=True)
 if group=='calibration':
  # Coverage is present in each of the other three frozen sequences.
  assert len(jobs)==4 and sum(not r['coverage_exception'] for r in collection_records if r['group']==group)==3

def cache_data(data,group):
 phase('extract_frozen_features_'+group)
 source=BallWMDataset(data,history_frames=5,rollout_horizon=5,image_size=(224,224),action_source='commanded_action',proprio_velocity_frame='base')
 assert len(source.episode_paths)==4
 checkpoint=torch.load(oldp['feature_checkpoint'],map_location='cpu',weights_only=False);assert sha(oldp['feature_checkpoint'])==oldp['feature_checkpoint_sha256']
 device=torch.device('cuda');encoder=build_joint_model(JointModelConfig(**checkpoint['config'])).to(device).eval();encoder.load_state_dict(checkpoint['model']);encoder.requires_grad_(False);embeddings=[]
 for ep in source.episode_paths:
  loaded=_load_episode(str(ep));s=loaded['states'];vectors=[]
  with torch.inference_mode():
   for start in range(0,loaded['num_frames'],16):
    frames=list(range(start,min(start+16,loaded['num_frames'])))
    inputs={f'pixels_{side}':torch.stack([_normalize_image(_load_image(ep/f'{side}_{frame:06d}.png',(224,224))) for frame in frames]).unsqueeze(0).to(device) for side in ['left','right']}
    ids=[5*(f+1)-1 for f in frames]
    inputs['proprio']=torch.from_numpy(np.stack([_proprio17(s,i,velocity_frame='base') for i in ids])).unsqueeze(0).to(device)
    inputs['action']=torch.from_numpy(np.stack([s['commanded_action'][i-4:i+1].reshape(10) for i in ids]).astype(np.float32)).unsqueeze(0).to(device)
    vectors.append(encoder.encode(inputs)['emb'][0].cpu().numpy())
  embeddings.append(np.concatenate(vectors));print('CACHED',group,str(ep),flush=True)
 del encoder;torch.cuda.empty_cache()
 windows=[];metadata=[]
 for ep_index,start in source.samples:
  ep=source.episode_paths[ep_index];loaded=_load_episode(str(ep));w=build_window(loaded['states'],embeddings[ep_index],start);windows.append(w);metadata.append({'episode':str(ep.resolve()),'scene':loaded['metadata']['scene'],'start_frame':int(start),'tags':tags(w)})
 tensors={k:torch.stack([w[k] for w in windows]) for k in windows[0]};assert len(metadata)==924
 cached={'tensors':tensors,'metadata':metadata,'partition':group,'feature_checkpoint_sha256':sha(oldp['feature_checkpoint'])};torch.save(cached,R/(group+'_features.pt'))
 fingerprints={str(f.relative_to(data)):sha(f) for f in sorted(Path(data).rglob('*')) if f.is_file()};dump(group+'_dataset_sha256.json',{'root':str(data),'files':fingerprints});return cached

def load_model(base,cls):
 m=cls();m.load_state_dict(torch.load(read(base/'checkpoint_selection.json')['checkpoint'],map_location='cpu',weights_only=False)['model']);return m.eval().requires_grad_(False)

def predict_report(model,cached,split):
 t=cached['tensors'];meta=cached['metadata'];ids=[i for i,m in enumerate(meta) if m['start_frame']%5==0]
 result=evaluate(model,t,ids,meta);result.update(split=split,gate_sha256=sha(R/'gate.pt'),historical_regression=split in ['val','test']);dump(split+'_prediction.json',result)
 with torch.inference_mode():coeff=model.gate({k:v[ids] for k,v in t.items()}).numpy()
 def usage(sub):return {f'{(h+1)*.1:.1f}':{'count':len(sub),'exact_cv_count':int(np.count_nonzero(coeff[sub,h]==0)),'mean_coefficient':float(coeff[sub,h].mean())} for h in range(5)}
 result_usage={'overall':usage(list(range(len(ids)))),'per_episode':{ep:usage([j for j,i in enumerate(ids) if meta[i]['episode']==ep]) for ep in sorted({meta[i]['episode'] for i in ids})}}
 dump(split+'_gate_usage.json',result_usage)
 # Full evaluated set is tested with labels absent and poisoned.
 b={k:v[ids] for k,v in t.items()}
 with torch.inference_mode():
  out=model(b);no_target=model({k:v for k,v in b.items() if k!='target'});poison=model({**b,'target':torch.full_like(b['target'],float('nan'))})
  assert torch.equal(out,no_target) and torch.equal(out,poison)
 return result

def historical_checks(reports):
 checks=[]
 for template in p['historical_check_templates']:
  x=dict(template);s=x['split'];d=reports[s];name=x['check']
  if name=='request_change_count':n=d['categories']['request_change']['window_count'];x.update(count=n,passed=n>=x['minimum'])
  elif name=='prefix_causality':delta=d['aggregate']['action_diagnostics']['prefix_pose_max_abs_delta'];x.update(delta=delta,passed=delta<=1e-5)
  else:
   h='0.1' if '0.1' in name else '0.5';key='yaw_abs_error_deg_mean' if name.endswith('yaw') else 'displacement_error_m_mean'
   if name.startswith('sequence_'):v=next(e for e in d['per_episode'] if e['episode']==x['episode'])['per_horizon'][h];denom=v['constant_velocity'][key]
   elif name.startswith('request_change_'):v=d['categories']['request_change']['per_horizon'][h];denom=v['constant_velocity'][key]
   else:
    v=d['aggregate']['per_horizon'][h]
    comparator=O if name.startswith('preserve_parent_') else (A if name.startswith('preserve_acceleration_') else None)
    denom=read(comparator/f'{s}_prediction.json')['aggregate']['per_horizon'][h]['model'][key] if comparator else v['constant_velocity'][key]
   ratio=v['model'][key]/denom;x.update(ratio=ratio,passed=ratio<=x['limit'])
  checks.append(x)
 return checks

def fresh_checks(report,accel,rate):
 checks=[]
 def ratio(name,value,denom,limit):r=value/denom;checks.append({'check':name,'ratio':r,'limit':limit,'passed':r<=limit})
 for h in ['0.1','0.5']:
  for axis,key in [('xy','displacement_error_m_mean'),('yaw','yaw_abs_error_deg_mean')]:
   v=report['aggregate']['per_horizon'][h];ratio('overall_'+h+'_'+axis,v['model'][key],v['constant_velocity'][key],.9 if h=='0.5' and axis=='xy' else 1.)
 for ep in report['per_episode']:
  v=ep['per_horizon']['0.5'];ratio('sequence_'+Path(ep['episode']).parent.name,v['model']['displacement_error_m_mean'],v['constant_velocity']['displacement_error_m_mean'],1.)
 sub=report['categories']['request_change'];n=sub['window_count'] if sub else 0;checks.append({'check':'request_change_count','count':n,'minimum':10,'passed':n>=10})
 for h in ['0.1','0.5']:
  if sub:v=sub['per_horizon'][h];ratio('request_change_'+h+'_xy',v['model']['displacement_error_m_mean'],v['constant_velocity']['displacement_error_m_mean'],1.)
 delta=report['aggregate']['action_diagnostics']['prefix_pose_max_abs_delta'];checks.append({'check':'prefix','delta':delta,'passed':delta<=1e-5})
 for name,other in [('acceleration',accel),('rate',rate)]:
  for axis,key in [('xy','displacement_error_m_mean'),('yaw','yaw_abs_error_deg_mean')]:ratio('preserve_'+name+'_'+axis,report['aggregate']['per_horizon']['0.5']['model'][key],other['aggregate']['per_horizon']['0.5']['model'][key],1.05)
 return checks

try:
 for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
 for f,h in p['input_sha256'].items():assert sha(f)==h,f
 assert sha(R/'confidence_gate.py')==p['gate_source_sha256'] and sha(R/'collection_manifest.json')==p['manifest_sha256']
 assert torch.cuda.is_available();dump('resume_runtime.json',{'torch':torch.__version__,'cuda':torch.version.cuda,'gpu':torch.cuda.get_device_name(0),'python':sys.executable,'environment':{k:env[k] for k in ['PYTHONPATH','NUMBA_CACHE_DIR','MPLCONFIGDIR']}})
 manifest=read(R/'collection_manifest.json');collect(manifest['jobs'],'calibration');cal=cache_data(Path(p['data_root']),'calibration')
 oldcache=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);assert not {m['episode'] for m in cal['metadata']} & {m['episode'] for m in oldcache['metadata']}
 base=load_model(A,AccelerationResidualPredictor);base_before={k:v.clone() for k,v in base.state_dict().items()};t=cal['tensors']
 phase('fit_identical_gate_new_calibration_only')
 with torch.inference_mode():prediction=base(t)
 labels,energy=scale_targets(prediction,t['prior'],t['target']);features=state_features(t);gate,history=fit_gate(features,labels,energy,steps=500)
 torch.save({'gate':gate.state_dict(),'protocol':p,'steps':500,'fit_episodes':sorted({m['episode'] for m in cal['metadata']})},R/'gate.pt');dump('calibration_history.json',history)
 assert all(torch.equal(v,base_before[k]) for k,v in base.state_dict().items());model=GatedPredictor(base,gate).eval()
 predict_report(model,cal,'calibration')
 phase('historical_evaluation')
 reports={}
 for split,ids in zip(['val','test'],oldcache['partitions'][1:]):
  sub={'tensors':{k:v[ids] for k,v in oldcache['tensors'].items()},'metadata':[oldcache['metadata'][i] for i in ids]};reports[split]=predict_report(model,sub,split)
  previous=read(A/f'{split}_prediction.json')
  assert [x['window_start_frames'] for x in reports[split]['per_episode']]==[x['window_start_frames'] for x in previous['per_episode']]
  for h in ['0.1','0.3','0.5']:
   for k in ['constant_velocity','zero_motion']:assert reports[split]['aggregate']['per_horizon'][h][k]==previous['aggregate']['per_horizon'][h][k]
 checks=historical_checks(reports);passed=all(x['passed'] for x in checks);dump('assessment.json',{'checks':checks,'passed':sum(x['passed'] for x in checks),'total':len(checks),'all_passed':passed,'historical_regression':True,'target_invariance':True,'base_weights_unchanged':True})
 print('HISTORICAL',sum(x['passed'] for x in checks),'/',len(checks),flush=True)
 fresh_result=None
 if passed:
  assert sha(p['fresh_manifest'])==p['fresh_manifest_sha256'];fresh=read(p['fresh_manifest']);collect(fresh['jobs'],'fresh')
  data=Path(fresh['jobs'][0]['output']).parents[1];cache=cache_data(data,'fresh');report=predict_report(model,cache,'fresh');ids=[i for i,m in enumerate(cache['metadata']) if m['start_frame']%5==0]
  accel=evaluate(base,cache['tensors'],ids,cache['metadata']);rate=evaluate(load_model(O,CVResidualPredictor),cache['tensors'],ids,cache['metadata']);dump('fresh_acceleration_prediction.json',accel);dump('fresh_rate_prediction.json',rate)
  fc=fresh_checks(report,accel,rate);fresh_result={'checks':fc,'all_passed':all(x['passed'] for x in fc),'passed':sum(x['passed'] for x in fc),'total':len(fc)};dump('fresh_assessment.json',fresh_result)
 else:
  assert all(not Path(j['output']).exists() for j in read(p['fresh_manifest'])['jobs']);print('FRESH_NOT_TRIGGERED',flush=True)
 for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
 for f,h in p['input_sha256'].items():assert sha(f)==h,f
 dump('disposition.json',{'historical_passed':passed,'fresh_run':fresh_result is not None,'fresh_passed':fresh_result['all_passed'] if fresh_result else None,'candidate_accepted_for_research':passed and fresh_result is not None and fresh_result['all_passed'],'navigation_runs':0,'new_calibration_sequences':4,'fresh_sequences':4 if fresh_result else 0})
 status.update(status='complete',phase='complete',returncode=0,ended_epoch=time.time());dump('resume_status.json',status)
except BaseException as ex:
 status.update(status='failed',error=f'{type(ex).__name__}: {ex}',ended_epoch=time.time());dump('resume_status.json',status);traceback.print_exc();raise
