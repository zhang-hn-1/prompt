"""Exactly one fit. Predict/seal all current causal inputs before scoring."""
from common import *
import numpy as np,torch,random
from torch.nn import functional as F
from model import PixelSurface
from torch.utils.data import Dataset,DataLoader
seed=20261013;random.seed(seed);np.random.seed(seed);torch.manual_seed(seed);torch.cuda.manual_seed_all(seed)
torch.set_num_threads(1);torch.backends.cudnn.benchmark=False
assert torch.cuda.is_available();device='cuda'
cov=json.loads((D/'coverage.json').read_text());assert cov['qualified']
verify_oracle();verify_hashes(cov['input_sha256']);verify_hashes(cov['output_sha256'])
mean=torch.tensor(cov['mean'],device=device)[None,:,None,None];std=torch.tensor(cov['std'],device=device)[None,:,None,None]
weight=torch.tensor(cov['class_weights'],device=device,dtype=torch.float32)
class Data(Dataset):
 def __init__(self):
  self.items=[]
  for group in ['H','E0']:
   for p in sorted((D/'features'/group).glob('*.npz')):
    f=np.load(p);s=np.load(D/'supervision'/group/p.name);self.items.append((f['x'],s['category'].astype(np.int64),np.nan_to_num(s['range_m'],nan=.5,posinf=.5,neginf=.5)))
 def __len__(self):return len(self.items)
 def __getitem__(self,i):return self.items[i]
model=PixelSurface().to(device);optimizer=torch.optim.Adam(model.parameters(),lr=.001);data=Data();assert len(data)==362
loader=DataLoader(data,batch_size=4,shuffle=True,num_workers=0,generator=torch.Generator().manual_seed(seed))
with (D/'fit_started.json').open('x') as marker:json.dump(dict(fit=1,seed=seed,protocol_sha256=sha(D/'protocol.json'),coverage_sha256=sha(D/'coverage.json')),marker,indent=2)
trace=[]
for epoch in range(30):
 totals=np.zeros(3);batches=0
 for x,label,z in loader:
  x=x.to(device);label=label.to(device);z=z.to(device);logit,depth=model((x-mean)/std)
  ce=F.cross_entropy(logit.reshape(-1,3,240,320),label.reshape(-1,240,320),weight=weight)
  parts=[]
  for kind in [1,2]:
   valid=label==kind
   if valid.any():parts.append(F.smooth_l1_loss(depth[valid],z[valid],beta=.1))
  distance=torch.stack(parts).mean();loss=ce+distance
  optimizer.zero_grad();loss.backward();optimizer.step();totals+=np.array([loss.item(),ce.item(),distance.item()]);batches+=1
 trace.append(dict(epoch=epoch+1,losses=(totals/batches).tolist()));print('epoch',epoch+1,trace[-1]['losses'],flush=True)
save('train_trace.json',trace);torch.save(dict(state_dict=model.state_dict(),mean=cov['mean'],std=cov['std'],protocol_sha256=sha(D/'protocol.json')),D/'model_final.pt')
model.eval();outputs={};inputs={}
verify_hashes(cov['output_sha256'])
with torch.no_grad():
 for group in GROUPS:
  (D/'predictions'/group).mkdir(parents=True,exist_ok=False)
  for j,p in enumerate(sorted((D/'features'/group).glob('*.npz'))):
   f=np.load(p);x=torch.from_numpy(f['x'])[None].to(device);logit,z=model((x-mean)/std)
   logits=F.interpolate(logit.reshape(1,6,240,320),size=(480,640),mode='bilinear',align_corners=False).reshape(1,2,3,480,640)
   prob=logits.softmax(2);confidence,category=prob.max(2);category=torch.where(confidence>=.9,category,0)
   depth=F.interpolate(z,size=(480,640),mode='bilinear',align_corners=False)
   out=D/'predictions'/group/p.name;np.savez_compressed(out,category=category[0].byte().cpu().numpy(),depth_m=depth[0].cpu().numpy(),confidence=confidence[0].cpu().numpy(),time_s=f['time_s'])
   outputs[str(out)]=sha(out);inputs[str(p)]=sha(p)
   if j%90==0:print('predict',group,j+1,flush=True)
save('prediction_seal.json',dict(outputs_sha256=outputs,inputs_sha256=inputs,model_sha256=sha(D/'model_final.pt'),source_sha256={str(p):sha(p) for p in [D/'model.py',D/'causal_input.py',D/'train_predict.py',D/'protocol.json']},pairs=len(outputs),fit=1,scope='Causal cached current input only; no labels, GT, scene or original SGBM read during prediction. Learned prior range/category, not binocular correspondence measurement.'))
