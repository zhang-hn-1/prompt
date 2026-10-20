def train_arm(name,cached,parts,epochs,initial):
 d=R/name;d.mkdir();t={k:v.cuda() for k,v in cached['tensors'].items()};meta=cached['metadata']
 random.seed(p['seed']);np.random.seed(p['seed']);torch.manual_seed(p['seed']);torch.cuda.manual_seed_all(p['seed'])
 model=AccelerationResidualPredictor().cuda();assert sum(x.numel() for x in model.parameters())==p['parameter_count']
 assert all(torch.equal(v.cpu(),initial[k]) for k,v in model.state_dict().items());model.load_state_dict(initial)
 with torch.inference_mode():assert torch.equal(model({k:v[:64] for k,v in t.items()}),t['prior'][:64])
 opt=torch.optim.AdamW(model.parameters(),lr=p['lr'],weight_decay=p['weight_decay']);history=[];total=0.;count=0;order_hash=hashlib.sha256();started=time.time()
 for step,ids in enumerate(schedule.tolist(),1):
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
