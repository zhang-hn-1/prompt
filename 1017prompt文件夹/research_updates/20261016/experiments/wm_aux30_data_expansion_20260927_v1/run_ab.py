"""Single fixed data expansion A/B. No gate, no tuning, no product-code changes."""
from pathlib import Path
import json,hashlib,time,random,traceback,ast
import numpy as np
import torch
from training_scope import batch_schedule,expanded_partition
from training.acceleration_residual import AccelerationResidualPredictor
from training.motion_residual import motion_loss
from scripts.run_wm_motion_residual import evaluate
R=Path(__file__).resolve().parent;read=lambda p:json.loads(Path(p).read_text());p=read(R/'protocol.json');ROOT=Path(p['root']);A=Path(p['original_run']);C=Path(p['expanded_data_run']);O=Path(p['frozen_rate_comparator'])
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def dump(name,x):(R/name).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
# Exact frozen scoring implementation, extracted without executing the old runner.
node=next(n for n in ast.parse((C/'pipeline.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='historical_checks')
exec(compile(ast.Module(body=[node],type_ignores=[]),str(C/'pipeline.py'),'exec'),globals())
assert not (R/'status.json').exists();status={'status':'running','started_epoch':time.time()}
def phase(s):status['phase']=s;dump('status.json',status);print(s,flush=True)
def check_inputs():
 for f,h in p['input_sha256'].items():assert sha(f)==h,f
 for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f

def train_arm(name,cached,parts,epochs,initial):
 d=R/name;d.mkdir();t={k:v.cuda() for k,v in cached['tensors'].items()};meta=cached['metadata']
 random.seed(p['seed']);np.random.seed(p['seed']);torch.manual_seed(p['seed']);torch.cuda.manual_seed_all(p['seed'])
 model=AccelerationResidualPredictor().cuda();assert sum(x.numel() for x in model.parameters())==p['parameter_count']
 assert all(torch.equal(v.cpu(),initial[k]) for k,v in model.state_dict().items());model.load_state_dict(initial)
 with torch.inference_mode():assert torch.equal(model({k:v[:64] for k,v in t.items()}),t['prior'][:64])
 opt=torch.optim.AdamW(model.parameters(),lr=p['lr'],weight_decay=p['weight_decay']);history=[];total=0.;count=0;order_hash=hashlib.sha256();started=time.time()
 for step,ids in enumerate(batch_schedule(parts[0],epochs,2,p['seed']),1):
  model.train();batch={k:v[ids] for k,v in t.items()};order_hash.update(np.asarray(ids,dtype=np.int64).tobytes());opt.zero_grad();loss=motion_loss(model(batch),batch['target']);assert torch.isfinite(loss);loss.backward();assert all(x.grad is None or torch.isfinite(x.grad).all() for x in model.parameters());opt.step();total+=float(loss.detach())*len(ids);count+=len(ids)
  if step%p['validation_interval_updates']==0:
   model.eval();val=0.
   with torch.inference_mode():
    for start in range(0,len(parts[1]),64):
     ix=parts[1][start:start+64];b={k:v[ix] for k,v in t.items()};val+=float(motion_loss(model(b),b['target']))*len(ix)
   row={'milestone':len(history),'optimizer_steps':step,'train_motion_loss':total/count,'val_motion_loss':val/len(parts[1])};history.append(row);total=0.;count=0
   torch.save({'model':model.state_dict(),'optimizer':opt.state_dict(),'metrics':row,'protocol':p},d/f'milestone{row["milestone"]:02d}.pt');dump(name+'/training_history.json',history);print(name,json.dumps(row),flush=True)
 assert step==27720 and len(history)==30
 selected=min(history,key=lambda x:(x['val_motion_loss'],x['optimizer_steps']));path=d/f'milestone{selected["milestone"]:02d}.pt';model.load_state_dict(torch.load(path,map_location='cuda',weights_only=False)['model'])
 dump(name+'/selection.json',{'selected':selected,'checkpoint':str(path),'checkpoint_sha256':sha(path),'training_seconds':time.time()-started,'order_sha256':order_hash.hexdigest(),'test_used_for_selection':False,'fit_episodes':sorted({meta[i]['episode'] for i in parts[0]}),'fit_windows':len(parts[0])})
 reports={}
 for split,ix in zip(['val','test'],parts[1:]):
  ids=[i for i in ix if meta[i]['start_frame']%5==0];report=evaluate(model,t,ids,meta);previous=read(A/f'{split}_prediction.json')
  assert [(e['episode'],e['window_start_frames']) for e in report['per_episode']]==[(e['episode'],e['window_start_frames']) for e in previous['per_episode']]
  for h in ['0.1','0.3','0.5']:
   for base in ['constant_velocity','zero_motion']:assert report['aggregate']['per_horizon'][h][base]==previous['aggregate']['per_horizon'][h][base]
  with torch.inference_mode():
   b={k:v[ids] for k,v in t.items()};out=model(b);assert torch.equal(out,model({k:v for k,v in b.items() if k!='target'}));assert torch.equal(out,model({**b,'target':torch.full_like(b['target'],float('nan'))}))
  report.update(historical_regression=True,checkpoint_sha256=sha(path),target_invariance=True);reports[split]=report;dump(name+'/'+split+'_prediction.json',report)
 checks=historical_checks(reports);dump(name+'/assessment.json',{'checks':checks,'passed':sum(x['passed'] for x in checks),'total':28,'all_passed':all(x['passed'] for x in checks)})
 return history,reports
try:
 phase('input_integrity');check_inputs()
 counts=[]
 for manifest,root in [(p['original_dataset_manifest'],p['original_data_root']),(p['expanded_dataset_manifest'],None)]:
  m=read(manifest);base=Path(root or m['root'])
  for f,h in m['files'].items():assert sha(base/f)==h,f
  counts.append(len(m['files']))
 old=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);new=torch.load(C/'calibration_features.pt',map_location='cpu',weights_only=False)
 assert old['feature_checkpoint_sha256']==new['feature_checkpoint_sha256']==p['original_feature_sha256']
 parts=expanded_partition(old['metadata'],old['partitions'],new['metadata']);combined={'tensors':{k:torch.cat([v,new['tensors'][k]]) for k,v in old['tensors'].items()},'metadata':old['metadata']+new['metadata']}
 assert [len(x) for x in parts]==[2772,462,462];assert len({combined['metadata'][i]['episode'] for i in parts[0]})==12
 dump('data_scope.json',{'original_raw_files_checked':counts[0],'new_raw_files_checked':counts[1],'original_partitions':list(map(len,old['partitions'])),'expanded_partitions':list(map(len,parts)),'complete_sequence_disjoint':True,'training_sequences':sorted({combined['metadata'][i]['episode'] for i in parts[0]})})
 assert torch.cuda.is_available();dump('runtime.json',{'torch':torch.__version__,'cuda':torch.version.cuda,'gpu':torch.cuda.get_device_name(0)})
 initial=torch.load(A/'initial_weights.pt',map_location='cpu',weights_only=False)
 phase('train_A_original');history,reports=train_arm('A_original',old,old['partitions'],30,initial)
 before=read(A/'training_history.json');delta=max(abs(x[k]-y[k]) for x,y in zip(history,before) for k in ['train_motion_loss','val_motion_loss']);assert delta<=1e-9,delta
 sel=read(R/'A_original/selection.json');assert sel['selected']['milestone']==read(A/'checkpoint_selection.json')['selected']['epoch']
 dump('reproduction.json',{'passed':True,'max_history_loss_delta':delta,'selected_original_epoch':sel['selected']['milestone'],'same_initial_weight_tensors':True})
 phase('train_B_expanded');history,reports=train_arm('B_expanded',combined,parts,20,initial)
 check_inputs();assessment=read(R/'B_expanded/assessment.json');fresh=read(p['fresh_manifest']);assert sha(p['fresh_manifest'])==p['fresh_manifest_sha256'];assert all(not Path(j['output']).exists() for j in fresh['jobs'])
 dump('disposition.json',{'historical_passed':assessment['all_passed'],'fresh_required':assessment['all_passed'],'fresh_run':False,'candidate_accepted':False,'navigation_runs':0,'reason':'Fresh required before acceptance' if assessment['all_passed'] else 'Historical gate failed; candidate rejected, no fresh collection'})
 status.update(status='complete',phase='historical_complete',ended_epoch=time.time());dump('status.json',status)
except BaseException as e:
 status.update(status='failed',error=f'{type(e).__name__}: {e}',ended_epoch=time.time());dump('status.json',status);traceback.print_exc();raise
