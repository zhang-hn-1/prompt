"""Single final-step decoder fit from OLD complete-sequence sources only."""
from pathlib import Path
import json,hashlib,datetime,time
import numpy as np
import torch
from fast_stereo import FastStereo
from dense_adapter import select_trainable,training_target,sequence_loss
from stereo_geometry import rectify_pair
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
 p=json.loads((H/'dense_adapter_fit_protocol.json').read_text());seal=json.loads((H/'dense_adapter_heldout_seal.json').read_text());start=now();assert seal['utc']<p['utc']<start
 assert not (H/'dense_adapter_fit_status.json').exists();assert sha(H/'dense_adapter_heldout_seal.json')==p['heldout_seal_sha256']
 for kind in ['source_sha256','input_sha256']:
  for f,d in p[kind].items():assert sha(f)==d,f
 for f,d in seal['artifact_sha256'].items():assert sha(f)==d,f
 status=dict(start_utc=start,status='running',fit_count=1,maximum_attempts=300);dest=H/'dense_adapter_fit_status.json';dest.write_text(json.dumps(status,indent=2)+'\n')
 torch.manual_seed(0);np.random.seed(0);torch.cuda.manual_seed_all(0)
 s=FastStereo();m=s.model;names=select_trainable(m);masks=np.load(p['mask']);examples=[]
 for row in p['examples']:
  rgb=np.load(row['rgb']);label=np.load(row['labels']);assert set(rgb.files)=={'left_rgb','right_rgb'} and set(label.files)=={'left_depth_m','right_depth_m'}
  eye=row['eye'];other='right' if eye=='left' else 'left';right=eye=='right'
  with np.errstate(invalid='ignore'):d,v=training_target(label[eye+'_depth_m'],label[other+'_depth_m'],masks[eye],masks[other],right_eye=right)
  assert v.sum()>100
  l,r=rectify_pair(rgb['left_rgb'],rgb['right_rgb'])
  if right:l,r=np.ascontiguousarray(r[:,::-1]),np.ascontiguousarray(l[:,::-1])
  examples.append((l,r,d,v,row,int(v.sum())))
 assert len(examples)==32
 frozen={n:hashlib.sha256(x.detach().cpu().contiguous().numpy().tobytes()).hexdigest() for n,x in m.state_dict().items() if n not in names}
 initial={n:hashlib.sha256(x.detach().cpu().contiguous().numpy().tobytes()).hexdigest() for n,x in m.named_parameters() if n in names}
 params=[x for x in m.parameters() if x.requires_grad];opt=torch.optim.AdamW(params,lr=1e-5,weight_decay=1e-4);scaler=torch.amp.GradScaler('cuda',init_scale=1.,growth_interval=1000)
 trace=[];clock=time.perf_counter()
 try:
  for i in range(300):
   l,r,d,v,row,count=examples[i%32];ts=[torch.from_numpy(a).cuda().float()[None].permute(0,3,1,2) for a in (l,r)];padder=s.Padder(ts[0].shape,divis_by=32,force_square=False);a,b=padder.pad(*ts)
   target=torch.from_numpy(d).cuda()[None,None];valid=torch.from_numpy(v).cuda()[None,None];opt.zero_grad(set_to_none=True)
   with torch.amp.autocast('cuda',dtype=torch.float16):
    _,pred=m(a,b,iters=8,test_mode=False,optimize_build_volume='pytorch1');loss=sequence_loss([padder.unpad(x) for x in pred],target,valid)
   if not bool(torch.isfinite(loss)):raise ValueError('nonfinite loss')
   scaler.scale(loss).backward();scaler.unscale_(opt)
   if any(x.grad is None or not bool(torch.isfinite(x.grad).all()) for x in params):raise ValueError('missing/nonfinite gradient')
   norm=float(torch.nn.utils.clip_grad_norm_(params,1.));scaler.step(opt);scaler.update()
   trace.append(dict(attempt=i+1,example=i%32,sequence=row['sequence'],frame=row['frame'],eye=row['eye'],valid_labels=count,loss=float(loss),unclipped_grad_norm=norm,scale=float(scaler.get_scale())))
   with (H/'dense_adapter_training_trace.jsonl').open('a') as f:f.write(json.dumps(trace[-1])+'\n')
   if (i+1)%10==0:print('FIT',i+1,float(loss),time.perf_counter()-clock,flush=True)
  state=m.state_dict();assert all(hashlib.sha256(state[n].detach().cpu().contiguous().numpy().tobytes()).hexdigest()==d for n,d in frozen.items())
  changed=[n for n,x in m.named_parameters() if n in names and hashlib.sha256(x.detach().cpu().contiguous().numpy().tobytes()).hexdigest()!=initial[n]];assert changed
  weights=H/'dense_adapter_parameters.pth';torch.save({n:state[n].detach().cpu().clone() for n in names},weights)
  status.update(end_utc=now(),status='finished',attempts=len(trace),optimizer_updates=len(trace),parameters_path=str(weights),parameters_sha256=sha(weights),changed_parameter_count=len(changed),trainable_names=names,frozen_state_exact=True,frozen_state_count=len(frozen),fit_wall_s=time.perf_counter()-clock,heldout_labels_decoded=False,all_state_not_learned_from_heldout=True,accepted=False,whole_research_complete=False)
 except BaseException as e:
  status.update(end_utc=now(),status='failed',attempts=len(trace),failure=repr(e),accepted=False,whole_research_complete=False);raise
 finally:dest.write_text(json.dumps(status,indent=2)+'\n')
if __name__=='__main__':main()
