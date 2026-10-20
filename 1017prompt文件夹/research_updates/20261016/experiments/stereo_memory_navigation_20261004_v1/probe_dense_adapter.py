"""ONE backwards feasibility probe, no optimizer/update/saved learned weights."""
from pathlib import Path
import json,hashlib,datetime,time
import numpy as np
import torch
from fast_stereo import FastStereo
from dense_adapter import select_trainable,training_target,sequence_loss
from stereo_geometry import rectify_pair
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
p=json.loads((H/'dense_adapter_probe_protocol.json').read_text())
for kind in ['source_sha256','input_sha256']:
 for f,d in p[kind].items():assert sha(f)==d,f
assert not (H/'dense_adapter_probe.json').exists()
rgb=np.load(p['rgb']);raw=np.load(p['truth']);mask=np.load(p['mask']);d,v=training_target(raw['left_depth_m'],raw['right_depth_m'],mask['left'],mask['right']);assert v.sum()>100
start=now();s=FastStereo();m=s.model;names=select_trainable(m)
old={n:hashlib.sha256(x.detach().cpu().contiguous().numpy().tobytes()).hexdigest() for n,x in m.state_dict().items()}
a,b=rectify_pair(rgb['left_rgb'],rgb['right_rgb']);ts=[torch.from_numpy(x).cuda().float()[None].permute(0,3,1,2) for x in (a,b)]
padder=s.Padder(ts[0].shape,divis_by=32,force_square=False);ta,tb=padder.pad(*ts)
target=torch.from_numpy(d).cuda()[None,None];valid=torch.from_numpy(v).cuda()[None,None]
with torch.amp.autocast('cuda',dtype=torch.float16):
 _,preds=m(ta,tb,iters=8,test_mode=False,optimize_build_volume='pytorch1');preds=[padder.unpad(o) for o in preds]
 loss=sequence_loss(preds,target,valid)
loss.backward();grads={n:None if x.grad is None else dict(finite=bool(torch.isfinite(x.grad).all()),norm=float(x.grad.float().norm())) for n,x in m.named_parameters() if x.requires_grad}
unchanged=all(hashlib.sha256(x.detach().cpu().contiguous().numpy().tobytes()).hexdigest()==old[n] for n,x in m.state_dict().items())
result=dict(start_utc=start,end_utc=now(),no_optimizer=True,parameter_update_count=0,all_state_unchanged=unchanged,trainable_names=names,trainable_parameters=sum(x.numel() for x in m.parameters() if x.requires_grad),total_parameters=sum(x.numel() for x in m.parameters()),valid_labels=int(v.sum()),loss=float(loss),gradient=grads,gpu_peak_bytes=int(torch.cuda.max_memory_allocated()),feasible=unchanged and all(g is not None and g['finite'] for g in grads.values()),protocol_sha256=sha(H/'dense_adapter_probe_protocol.json'))
(H/'dense_adapter_probe.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('gradient','trainable_names')},indent=2))
