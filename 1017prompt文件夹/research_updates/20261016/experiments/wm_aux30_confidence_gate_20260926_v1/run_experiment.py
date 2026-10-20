"""One fixed-budget confidence-scaling candidate; historical data only."""
from pathlib import Path
import json,hashlib,time,traceback,sys
import numpy as np
import torch
from confidence_gate import state_features,scale_targets,fit_gate,GatedPredictor
from training.acceleration_residual import AccelerationResidualPredictor
from scripts.run_wm_motion_residual import evaluate
R=Path(__file__).resolve().parent
ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
A=ROOT/'research/ball_wm/runs/wm_aux30_acceleration_20260924_v1'
read=lambda p:json.loads(Path(p).read_text())
def dump(name,value):(R/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
assert not (R/'status.json').exists(),'Refuse to overwrite previous experiment'
oldp=read(A/'protocol.json');O=Path(oldp['parent_run']);selection=read(A/'checkpoint_selection.json')
inputs={}
for base in [A,O]:
 for file in base.iterdir():
  if file.is_file() and file.suffix in ['.json','.pt','.log','.py','.png']:inputs[str(file)]=sha(file)
inputs[str(ROOT/'docs/world_model_aux30_acceleration_residual_20260924.md')]=sha(ROOT/'docs/world_model_aux30_acceleration_residual_20260924.md')
protocol={'created_epoch':time.time(),'root':str(ROOT),'output':str(R),'frozen_acceleration_run':str(A),'frozen_rate_comparator':str(O),
 'candidate_count':1,'features':'current proprio17 and current-minus-oldest proprio17 in 5-frame history; state trends, not inertial acceleration; no future requests/feedback/images used by gate',
 'parameters':175,'coefficient':'sigmoid(Linear34to5(train-standardized-state)); scaling coefficient, not calibrated probability',
 'fit':'Only 1848 windows from eight original train sequences. Per-horizon alpha*=clip(c dot r / c dot c,0,1); XY m and wrapped yaw rad, coefficient 1. Energy weights c dot c normalized per horizon. Zero energy <=1e-12 receives no fit weight.',
 'normalization':'training mean/population std only, scale floored1e-4',
 'optimizer':{'name':'Adam','lr':.01,'steps':500,'full_batch':True,'l2_weight_mean_square':.01,'initialization':'all weights/biases zero','checkpoint':'last fixed step; no validation/test selection'},
 'deployment':'coefficient < .5 -> zero/exact constant velocity; otherwise multiply frozen residual by coefficient. Same scalar for XY and shortest yaw angle per horizon.',
 'fit_deployment_difference':'Fit smooth coefficients, apply fixed .5 cutoff at evaluation. Clipped target regression is calibration, not exact shared-parameter original pose-risk minimization.',
 'historical_scope':'Validation and test already influenced design; whole-sequence historical regression. Base-model training residuals are in-sample, not uncertainty calibration on fresh data.',
 'historical_gate':oldp['historical_gate'],'acceleration_acceptance':oldp['acceleration_acceptance'],
 'extra_acceptance':{'each_val_and_test_0.5_xy_ratio_to_frozen_acceleration_max':1.05,'each_val_and_test_0.5_yaw_ratio_to_frozen_acceleration_max':1.05},
 'navigation':'no new navigation or collection in this bounded calibration test; any accepted result still needs fresh whole-sequence validation',
 'source_sha256':oldp['frozen_sources'],'input_sha256':inputs,'new_code_sha256':{f:sha(R/f) for f in ['confidence_gate.py','test_confidence_gate.py','run_experiment.py']}}
dump('protocol.json',protocol);status={'status':'running','started_epoch':time.time(),'phase':'verify'};dump('status.json',status)
try:
 for f,h in protocol['source_sha256'].items():assert sha(ROOT/f)==h,f
 c=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);t=c['tensors'];metadata=c['metadata'];parts=c['partitions'];train=parts[0]
 assert list(map(len,parts))==[1848,462,462]
 base=AccelerationResidualPredictor();base.load_state_dict(torch.load(selection['checkpoint'],map_location='cpu',weights_only=False)['model']);base.eval().requires_grad_(False)
 base_before={k:v.clone() for k,v in base.state_dict().items()}
 train_batch={k:v[train] for k,v in t.items()}
 with torch.inference_mode():pred=base(train_batch)
 label,energy=scale_targets(pred,train_batch['prior'],train_batch['target']);features=state_features(train_batch)
 # Fit API receives only train tensors; neither held-out normalization nor labels are passed.
 status['phase']='fit175parameters500steps';dump('status.json',status)
 gate,history=fit_gate(features,label,energy,steps=500)
 torch.save({'gate':gate.state_dict(),'protocol':protocol,'steps':500},R/'gate.pt');dump('calibration_history.json',history)
 assert all(torch.equal(v,base_before[k]) for k,v in base.state_dict().items())
 torch.testing.assert_close(gate.feature_mean,features.mean(0),rtol=0,atol=0)
 model=GatedPredictor(base,gate).eval()
 # Exercise complete forward with labels removed and poisoned; labels cannot affect inference.
 probe={k:v[:12] for k,v in t.items()};without={k:v for k,v in probe.items() if k!='target'}
 with torch.inference_mode():
  original=model(probe);no_target=model(without);poison={**probe,'target':torch.full_like(probe['target'],1e6)}
  assert torch.equal(original,no_target) and torch.equal(original,model(poison))
 gate_stats={};reports={}
 for split,ids in zip(['train','val','test'],parts):
  selected_ids=[i for i in ids if metadata[i]['start_frame']%5==0]
  batch={k:v[selected_ids] for k,v in t.items()}
  with torch.inference_mode():coeff=gate(batch).numpy();raw=gate.coefficients(state_features(batch),apply_cutoff=False).numpy()
  def stats(indices):
   return {f'{(h+1)*.1:.1f}':{'exact_cv_count':int(np.count_nonzero(coeff[indices,h]==0)),'count':len(indices),'mean_coefficient':float(coeff[indices,h].mean()),'mean_raw_coefficient':float(raw[indices,h].mean())} for h in range(5)}
  gate_stats[split]={'overall':stats(list(range(len(selected_ids)))),'per_episode':{ep:stats([j for j,i in enumerate(selected_ids) if metadata[i]['episode']==ep]) for ep in sorted({metadata[i]['episode'] for i in selected_ids})}}
  if split=='train':continue
  status['phase']='evaluate_'+split;dump('status.json',status)
  report=evaluate(model,t,selected_ids,metadata);report.update(split=split,gate_checkpoint=str(R/'gate.pt'),gate_sha256=sha(R/'gate.pt'),base_checkpoint=selection['checkpoint'],historical_regression=True)
  old=read(A/f'{split}_prediction.json')
  assert report['aggregate']['window_count']==old['aggregate']['window_count']==94
  assert [e['window_start_frames'] for e in report['per_episode']]==[e['window_start_frames'] for e in old['per_episode']]
  for h in ['0.1','0.3','0.5']:
   for baseline in ['constant_velocity','zero_motion']:assert report['aggregate']['per_horizon'][h][baseline]==old['aggregate']['per_horizon'][h][baseline]
  reports[split]=report;dump(f'{split}_prediction.json',report)
 dump('gate_usage.json',gate_stats)
 # Reapply identical original 24 checks with original rate parent comparator.
 checks=[]
 for template in read(A/'assessment.json')['checks']:
  item={k:v for k,v in template.items() if k not in ['passed','ratio','delta','count']};s=item['split'];report=reports[s];check=item['check'];limit=item.get('limit')
  if check=='request_change_count':
   count=report['categories']['request_change']['window_count'];item.update(count=count,passed=count>=item['minimum'])
  elif check=='prefix_causality':
   delta=report['aggregate']['action_diagnostics']['prefix_pose_max_abs_delta'];item.update(delta=delta,passed=delta<=oldp['historical_gate']['prefix_max_abs_tolerance'])
  else:
   key='yaw_abs_error_deg_mean' if check.endswith('yaw') else 'displacement_error_m_mean'
   h='0.1' if '0.1' in check else '0.5'
   if check.startswith('sequence_'):
    data=next(e for e in report['per_episode'] if e['episode']==item['episode'])['per_horizon'][h];denom=data['constant_velocity'][key]
   elif check.startswith('request_change_'):
    data=report['categories']['request_change']['per_horizon'][h];denom=data['constant_velocity'][key]
   else:
    data=report['aggregate']['per_horizon'][h]
    denom=read(O/f'{s}_prediction.json')['aggregate']['per_horizon'][h]['model'][key] if check.startswith('preserve_parent_') else data['constant_velocity'][key]
   ratio=data['model'][key]/denom;item.update(ratio=ratio,passed=ratio<=limit)
  checks.append(item)
 for s in ['val','test']:
  for name,key in [('xy','displacement_error_m_mean'),('yaw','yaw_abs_error_deg_mean')]:
   current=reports[s]['aggregate']['per_horizon']['0.5']['model'][key];before=read(A/f'{s}_prediction.json')['aggregate']['per_horizon']['0.5']['model'][key];ratio=current/before
   checks.append({'split':s,'gate':'confidence_preservation','check':'preserve_acceleration_0.5_'+name,'ratio':ratio,'limit':1.05,'passed':ratio<=1.05})
 dump('assessment.json',{'checks':checks,'all_passed':all(x['passed'] for x in checks),'original_checks':24,'additional_checks':4,'passed':sum(x['passed'] for x in checks),'future_target_invariance':True,'base_weights_unchanged':True,'training_sequences':sorted({metadata[i]['episode'] for i in train}),'gate_parameter_count':sum(x.numel() for x in gate.parameters())})
 assert all(sha(f)==h for f,h in inputs.items())
 status.update(status='complete',phase='complete',ended_epoch=time.time(),returncode=0);dump('status.json',status)
 print(json.dumps({'checks_passed':sum(x['passed'] for x in checks),'total_checks':len(checks),'failed':[x for x in checks if not x['passed']],'runtime_s':status['ended_epoch']-status['started_epoch'],'usage':gate_stats['test']['overall']},indent=2))
except BaseException as ex:
 status.update(status='failed',error=str(ex),ended_epoch=time.time());dump('status.json',status);raise
