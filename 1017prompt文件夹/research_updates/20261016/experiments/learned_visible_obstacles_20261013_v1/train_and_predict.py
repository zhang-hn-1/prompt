from pathlib import Path
import os,json,hashlib,random,time
import numpy as np,torch
from torch.nn import functional as F
from model import VisibleMapNet,tentative
D=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();c=json.loads((D/'protocol.json').read_text());q=json.loads((D/'coverage.json').read_text());assert q['pass_coverage'];review=json.loads((D/'prefit_review.json').read_text());assert review['admit_fit'],review
for group in ['dataset_sha256','source_sha256']:
 for p,s in q[group].items():assert sha(Path(p))==s
for p,s in review['reviewed_source_sha256'].items():assert sha(Path(p))==s
for row in json.loads((D/'inference_inputs.json').read_text()):assert sha(D/'inference_inputs'/f"{row['name']}.npz")==row['sha256']
receipt=D/'fit_started.json'
with receipt.open('x') as f:json.dump(dict(fit=1,seed=2026,epochs=30,train_groups=['H','E0'],fit_excluded='E1',start=time.time(),protocol_sha=sha(D/'protocol.json'),source_sha={str(p):sha(p) for p in [D/'model.py',Path(__file__)]}),f,indent=2)
random.seed(2026);np.random.seed(2026);torch.manual_seed(2026);torch.cuda.manual_seed_all(2026);torch.set_num_threads(2);torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True;assert torch.cuda.is_available();dev='cuda'
X=[];G=[];Y=[]
for name in ['H','E0']:
 with np.load(D/'data'/f'{name}.npz') as a:X.append(a['rgb']);G.append(a['gravity']);Y.append(a['labels'])
x=torch.from_numpy(np.concatenate(X)).float().div_(255).to(dev);g=torch.from_numpy(np.concatenate(G)).to(dev);y=torch.from_numpy(np.concatenate(Y)).long().to(dev);counts=torch.bincount(y.flatten(),minlength=5).float();w=counts.clamp_min(1).rsqrt();w=w/w.mean();m=VisibleMapNet().to(dev);opt=torch.optim.Adam(m.parameters(),lr=.001);trace=[]
for epoch in range(30):
 m.train();order=torch.randperm(len(x),device=dev);losses=[]
 for start in range(0,len(x),16):
  idx=order[start:start+16];opt.zero_grad(set_to_none=True);z=m(x[idx],g[idx]);loss=F.cross_entropy(z,y[idx],weight=w);loss.backward();opt.step();losses.append(float(loss.detach()))
 row=dict(epoch=epoch+1,train_loss=float(np.mean(losses)));trace.append(row);print(row,flush=True)
checkpoint=D/'model_final.pt';torch.save(dict(state_dict=m.cpu().state_dict(),class_weights=w.cpu(),seed=2026,epoch=30,protocol_sha=sha(D/'protocol.json'),train_dataset_sha={str(D/'data'/f'{n}.npz'):sha(D/'data'/f'{n}.npz') for n in ['H','E0']}),checkpoint);(D/'train_trace.json').write_text(json.dumps(trace,indent=2)+'\n');m.to(dev).eval()
# Held-group inference opens sanitized RGB+gravity file only; no evaluation labels.
out=D/'predictions';out.mkdir(exist_ok=False)
with torch.no_grad():
 for name in ['H','E0','E1']:
  a=np.load(D/'inference_inputs'/f'{name}.npz');assert set(a.files)=={'rgb','gravity'};ps=[];ls=[]
  for start in range(0,len(a['rgb']),16):
   z=m(torch.from_numpy(a['rgb'][start:start+16]).float().div_(255).to(dev),torch.from_numpy(a['gravity'][start:start+16]).to(dev));label,prob=tentative(z);ls.append(label.cpu().numpy());ps.append(prob.cpu().numpy().astype(np.float32))
  np.savez_compressed(out/f'{name}.npz',labels=np.concatenate(ls),prob=np.concatenate(ps))
seal=dict(checkpoint_sha=sha(checkpoint),predictions={str(p):sha(p) for p in out.glob('*.npz')},inputs={str(p):sha(p) for p in (D/'inference_inputs').glob('*.npz')},train_trace_sha=sha(D/'train_trace.json'),fit=1,epochs=30,no_E1_loss_or_selection=True,source_sha={str(p):sha(p) for p in [D/'model.py',Path(__file__),D/'protocol.json']});(D/'prediction_seal.json').write_text(json.dumps(seal,indent=2)+'\n');print('final checkpoint and all predictions sealed',flush=True)
