"""Read-only paired error localization; no model fitting, selection, or control."""
from pathlib import Path
import json,hashlib,time,math
import numpy as np
import torch
from training.acceleration_residual import AccelerationResidualPredictor
from training.pose_utils import planar_yaw_from_wxyz
S=Path(__file__).resolve().parent;ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');RUNS=ROOT/'research/ball_wm/runs';AB=RUNS/'wm_aux30_data_expansion_20260927_v1';A=RUNS/'wm_aux30_acceleration_20260924_v1';C=RUNS/'wm_aux30_independent_calibration_20260926_v1'
read=lambda f:json.loads(Path(f).read_text())
def dump(name,obj):(S/name).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def sha(f):return hashlib.sha256(Path(f).read_bytes()).hexdigest()
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def changed(q):
 d=np.abs(np.diff(q,axis=0));return np.flatnonzero((d[:,0]>=.1-1e-6)|(d[:,1]>=np.deg2rad(5)-1e-6))+1

def stop_qualified(speed,anchor):
 indices=anchor-5*np.arange(4,-1,-1)
 return bool(indices[0]>=0 and np.all(speed[indices]<.05))

def summarize(rows):
 if not rows:return {'count':0}
 out={'count':len(rows),'horizons':{}}
 for h in ['0.1','0.3','0.5']:
  out['horizons'][h]={}
  for model in ['A','B','CV']:
   vs=[r['h'][h][model] for r in rows];out['horizons'][h][model]={k:float(np.mean([v[k] for v in vs])) for k in ['xy_cm','yaw_deg','yaw_signed_deg','xy_mse','yaw_mse']}
   if model!='CV':out['horizons'][h][model].update(yaw_wrong_correction_direction=sum(v['yaw_needed_deg']*v['yaw_correction_deg']<0 for v in vs),yaw_worse_than_cv=sum(v['yaw_deg']>r['h'][h]['CV']['yaw_deg'] for v,r in zip(vs,rows)),xy_wrong_correction_direction=sum(v['xy_dot']<0 for v in vs),xy_worse_than_cv=sum(v['xy_cm']>r['h'][h]['CV']['xy_cm'] for v,r in zip(vs,rows)),mean_abs_yaw_correction_deg=float(np.mean([abs(v['yaw_correction_deg']) for v in vs])),mean_abs_yaw_needed_deg=float(np.mean([abs(v['yaw_needed_deg']) for v in vs])))
  out['horizons'][h]['B_minus_A_yaw_error_sum_deg']=sum(r['h'][h]['B']['yaw_deg']-r['h'][h]['A']['yaw_deg'] for r in rows)
 return out

# Diagnostic self checks: wrapping, actual five-sample stop and bounce reset, threshold timing.
assert np.isclose(np.rad2deg(wrap(np.deg2rad(181))),-179)
assert np.isclose(np.rad2deg(wrap(np.deg2rad(-181))),179)
speed=np.zeros(30);assert stop_qualified(speed,24);speed[14]=.05;assert not stop_qualified(speed,24)
assert not stop_qualified(np.zeros(15),14)
assert changed(np.array([[0.,0.],[0.,0.],[.2,0.],[.2,.2]])).tolist()==[2,3]
assert 5*(35+5)-1==199 and (200-199-1)*.02==0
assert not (S/'results.json').exists(),'Refuse overwrite'
p=read(AB/'protocol.json');old=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);extra=torch.load(C/'calibration_features.pt',map_location='cpu',weights_only=False);metadata=old['metadata']+extra['metadata'];t={k:torch.cat([v,extra['tensors'][k]]) for k,v in old['tensors'].items()}
parts={name:list(ids) for name,ids in zip(['old_fit','val','test'],old['partitions'])};parts['new_fit']=list(range(len(old['metadata']),len(metadata)))
inputs={str(f):sha(f) for f in [A/'feature_windows.pt',C/'calibration_features.pt',AB/'protocol.json',AB/'reproduction.json']}
models={}
for label,arm in [('A','A_original'),('B','B_expanded')]:
 info=read(AB/arm/'selection.json');f=Path(info['checkpoint']);assert sha(f)==info['checkpoint_sha256'];inputs[str(f)]=sha(f);inputs[str(AB/arm/'selection.json')]=sha(AB/arm/'selection.json')
 model=AccelerationResidualPredictor();model.load_state_dict(torch.load(f,map_location='cpu',weights_only=False)['model']);model.eval();models[label]=model
 for split in ['val','test']:inputs[str(AB/arm/f'{split}_prediction.json')]=sha(AB/arm/f'{split}_prediction.json')
protocol={'created_before_analysis_epoch':time.time(),'scope':'Post-hoc paired historical diagnosis; no fit, checkpoint selection, simulation or navigation','inputs_sha256':inputs,'source_sha256':p['source_sha256'],'sampling':'Original stride5 frames for primary metrics; all cached anchors only for event alignment. old_fit/new_fit are in-sample for B.','states':'speed XY bins <.05 / .05-.15 / >=.15m/s; root-yaw backward .1s finite-difference turning threshold abs>=5deg/s; measured aux abs bins <5 /5-15 />=15deg; actual motion sign uses instantaneous yaw-forward velocity +/- .05m/s','stop':'Five consecutive past10Hz XY speeds <.05m/s, measured not command. Any sample >=.05 resets eligibility.','future_tags':'Actual velocity reversal within .5s is retrospective label only; never an input. Command timing uses post-step states and next interval at request[j].','event_offsets_s':[-.5,-.1,0,.1,.5],'errors':'Signed wrapped yaw, XY norm; decompose corrections relative to same CV; no future-label input','gradient_diagnostic':'Shared context+GRU gradients of separate XY m2 and yaw rad2 terms at frozen selected weights; full original/new training windows. Local cosine not proof of past optimization causality.','acceptance':'Unchanged24/28 failure from parent; no new gates','script_sha256':sha(__file__)};dump('protocol.json',protocol)
state={};rawchecks=0
for ep in sorted({m['episode'] for m in metadata}):
 f=Path(ep)/'states.npz';inputs[str(f)]=sha(f)
 with np.load(f) as z:s={k:z[k] for k in z.files}
 yaw=np.array([planar_yaw_from_wxyz(q) for q in s['quat']]);speed=np.linalg.norm(s['lin_vel'][:,:2],axis=1);forward=s['lin_vel'][:,0]*np.cos(yaw)+s['lin_vel'][:,1]*np.sin(yaw);ev=changed(s['commanded_action']);state[ep]=(s,yaw,speed,forward,ev)
 assert len(yaw)==1200 and np.array_equal(s['physics_step'],np.arange(1,1201));rawchecks+=1
preds={}
with torch.inference_mode():
 for name,model in models.items():preds[name]=torch.cat([model({k:v[start:start+64] for k,v in t.items()}) for start in range(0,len(metadata),64)]).numpy().astype(float)
rows=[];lookup={};metric_delta=0.
for split,ids in parts.items():
 for i in ids:
  m=metadata[i];s,yaw,speed,forward,ev=state[m['episode']];anchor=5*(m['start_frame']+5)-1;future=ev[(ev>anchor)&(ev<=anchor+25)];early=future[future<=anchor+5];past=ev[ev<=anchor];dt=.02
  assert np.array_equal(t['requests'][i].numpy(),s['commanded_action'][anchor+1:anchor+26].reshape(5,10))
  assert np.allclose(t['target'][i,:,2:].numpy(),np.stack([np.sin(yaw[anchor+5*np.arange(1,6)]-yaw[anchor]),np.cos(yaw[anchor+5*np.arange(1,6)]-yaw[anchor])],axis=1),atol=1e-6)
  rate=float(np.rad2deg(wrap(yaw[anchor]-yaw[anchor-5]))/.1);aux=float(np.rad2deg(s['joint_pos'][anchor,1]));current=float(forward[anchor]);rev=bool((current>.05 and np.any(forward[anchor+1:anchor+26]<-.05)) or (current<-.05 and np.any(forward[anchor+1:anchor+26]>.05)))
  groups={'speed':'slow' if speed[anchor]<.05 else ('medium' if speed[anchor]<.15 else 'fast'),'turning':'turning' if abs(rate)>=5 else 'low_yaw_rate','aux':'small' if abs(aux)<5 else ('middle' if abs(aux)<15 else 'large'),'motion':'forward' if current>.05 else ('reverse' if current<-.05 else 'near_zero_forward_component'),'stop':'qualified' if stop_qualified(speed,anchor) else 'not_qualified','request_timing':'first_0.1_change' if len(early) else ('later_only_change' if len(future) else 'no_future_change'),'actual_future_reversal':'yes' if rev else 'no'}
  row={'split':split,'episode':m['episode'],'start_frame':m['start_frame'],'anchor_s':float(s['state_timestamp'][anchor]),'primary':m['start_frame']%5==0,'request_change':bool(len(future)),'speed_m_s':float(speed[anchor]),'yaw_rate_deg_s':rate,'world_z_ang_vel_deg_s':float(np.rad2deg(s['ang_vel'][anchor,2])),'actual_aux_deg':aux,'actual_aux_vel_rad_s':float(s['joint_vel'][anchor,1]),'first_request_change_offset_s':float((future[0]-anchor-1)*dt) if len(future) else None,'last_change_age_s':float((anchor-past[-1]+1)*dt) if len(past) else None,'groups':groups,'h':{}}
  for step in [0,2,4]:
   h=f'{(step+1)*.1:.1f}';target=t['target'][i,step].numpy().astype(float);prior=t['prior'][i,step].numpy().astype(float);truthyaw=np.arctan2(target[2],target[3]);prioryaw=np.arctan2(prior[2],prior[3]);row['h'][h]={}
   for model in ['A','B','CV']:
    pp=prior if model=='CV' else preds[model][i,step];pyaw=np.arctan2(pp[2],pp[3]);error=float(wrap(pyaw-truthyaw));c=pp[:2]-prior[:2];needed=target[:2]-prior[:2];xy=float(np.linalg.norm(pp[:2]-target[:2]));row['h'][h][model]={'xy_cm':100*xy,'yaw_deg':float(abs(np.rad2deg(error))),'yaw_signed_deg':float(np.rad2deg(error)),'xy_mse':xy**2,'yaw_mse':error**2,'yaw_needed_deg':float(np.rad2deg(wrap(truthyaw-prioryaw))),'yaw_correction_deg':float(np.rad2deg(wrap(pyaw-prioryaw))),'xy_dot':float(c@needed)}
  lookup[(m['episode'],m['start_frame'])]=row;rows.append(row)
summary={};coverage={};per_episode={}
for split in parts:
 primary=[r for r in rows if r['split']==split and r['primary']];summary[split]={'all':summarize(primary),'request_change':summarize([r for r in primary if r['request_change']]),'groups':{}};coverage[split]={}
 for key in primary[0]['groups']:
  summary[split]['groups'][key]={v:summarize([r for r in primary if r['groups'][key]==v]) for v in sorted({r['groups'][key] for r in primary})};coverage[split][key]={v:x['count'] for v,x in summary[split]['groups'][key].items()}
 per_episode[split]={ep:summarize([r for r in primary if r['episode']==ep]) for ep in sorted({r['episode'] for r in primary})}
 if split in ['val','test']:
  for model,arm in [('A','A_original'),('B','B_expanded')]:
   original=read(AB/arm/f'{split}_prediction.json')
   for h in ['0.1','0.3','0.5']:
    for ours,theirs,factor in [('xy_cm','displacement_error_m_mean',100),('yaw_deg','yaw_abs_error_deg_mean',1)]:
     d=abs(summary[split]['all']['horizons'][h][model][ours]-original['aggregate']['per_horizon'][h]['model'][theirs]*factor);metric_delta=max(metric_delta,d);assert d<1e-5
# Event-aligned anchors, respecting start-of-execution interval, no plan substitution.
event_rows=[];events_count={}
for split in ['val','test']:
 eps=sorted(per_episode[split]);events_count[split]=0
 for ep in eps:
  ev=state[ep][-1]
  for index in ev:
   events_count[split]+=1;start=(int(index)-25)//5;assert 5*start+24==index-1
   for offset in [-5,-1,0,1,5]:
    if (ep,start+offset) in lookup:event_rows.append({'split':split,'event_tick':int(index),'offset_s':offset*.1,'row':lookup[(ep,start+offset)]})
aligned={split:{f'{offset:.1f}':summarize([x['row'] for x in event_rows if x['split']==split and np.isclose(x['offset_s'],offset)]) for offset in [-.5,-.1,0,.1,.5]} for split in ['val','test']}
# Gradient diagnostic only, no optimizer or weight mutation.
gradients={}
for name,model in models.items():
 shared=[v for key,v in model.named_parameters() if not key.startswith('readout.')];gradients[name]={}
 for split in ['old_fit','new_fit']:
  ids=parts[split];gx=[torch.zeros_like(v) for v in shared];gy=[torch.zeros_like(v) for v in shared];losses=np.zeros(2)
  for start in range(0,len(ids),64):
   ix=ids[start:start+64];b={k:v[ix] for k,v in t.items()};pr=model(b)[:,[0,2,4]];ta=b['target'][:,[0,2,4]];lx=(pr[...,:2]-ta[...,:2]).square().sum(-1).mean();angle=torch.atan2(pr[...,2]*ta[...,3]-pr[...,3]*ta[...,2],pr[...,2]*ta[...,2]+pr[...,3]*ta[...,3]);ly=angle.square().mean();weight=len(ix)/len(ids)
   for dest,loss,retain in [(gx,lx,True),(gy,ly,False)]:
    values=torch.autograd.grad(loss,shared,retain_graph=retain)
    for z,v in zip(dest,values):z.add_(v.detach(),alpha=weight)
   losses+=np.array([float(lx.detach()),float(ly.detach())])*weight
  x=torch.cat([v.flatten() for v in gx]).double();y=torch.cat([v.flatten() for v in gy]).double();gradients[name][split]={'count':len(ids),'xy_loss_m2':float(losses[0]),'yaw_loss_rad2':float(losses[1]),'yaw_fraction_of_loss':float(losses[1]/sum(losses)),'shared_xy_grad_norm':float(x.norm()),'shared_yaw_grad_norm':float(y.norm()),'shared_xy_yaw_cosine':float(torch.dot(x,y)/(x.norm()*y.norm()))}
for f,h in inputs.items():assert sha(f)==h,f
for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
assert all(not Path(j['output']).exists() for j in read(p['fresh_manifest'])['jobs'])
top={split:sorted([r for r in rows if r['split']==split and r['primary']],key=lambda r:r['h']['0.5']['B']['yaw_deg']-r['h']['0.5']['A']['yaw_deg'],reverse=True)[:10] for split in ['val','test']}
result={'status':'complete','input_files_unchanged':len(inputs),'source_files_unchanged':len(p['source_sha256']),'raw_sequence_alignment_checks':rawchecks,'self_checks':7,'max_metric_reproduction_delta':metric_delta,'summary':summary,'coverage':coverage,'per_episode':per_episode,'event_counts':events_count,'event_aligned':aligned,'gradient_diagnostic':gradients,'top_worsened_yaw':top,'fresh_untouched':True,'training_runs':0,'simulation_runs':0,'prior_gate_unchanged':True}
dump('input_hashes.json',inputs);dump('rows.json',rows);dump('event_rows.json',event_rows);dump('results.json',result)
print(json.dumps({'status':'complete','event_counts':events_count,'max_metric_reproduction_delta':metric_delta,'gradient_diagnostic':gradients},indent=2))
