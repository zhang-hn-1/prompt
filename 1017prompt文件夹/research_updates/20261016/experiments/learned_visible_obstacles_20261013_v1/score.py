from pathlib import Path
import json,hashlib,numpy as np
from scipy.ndimage import maximum_filter,distance_transform_edt
D=Path(__file__).resolve().parent;seal=json.loads((D/'prediction_seal.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for group in ['predictions','source_sha']:
 for p,s in seal[group].items():assert sha(p)==s
assert sha(D/'model_final.pt')==seal['checkpoint_sha']
for p,s in json.loads((D/'coverage.json').read_text())['dataset_sha256'].items():assert sha(p)==s
train=np.concatenate([np.load(D/'data'/f'{n}.npz')['labels'] for n in ['H','E0']]);counts=np.stack([(train==k).sum(0) for k in range(5)]);prior=counts.argmax(0)
def metrics(Y,P):
 n_pred=n_gt=matched_pred=matched_gt=ground=falseground=unsupported=visible=recalledvisible=0;dist=[];heightok=heightn=0
 for y,p in zip(Y,P):
  risk=p>=2;gt=y>=2;near_gt=maximum_filter(gt,size=3,mode='constant');near_pred=maximum_filter(risk,size=3,mode='constant');vis=y>0;n_pred+=int(risk.sum());n_gt+=int(gt.sum());matched_pred+=int((risk&near_gt).sum());matched_gt+=int((gt&near_pred).sum());ground+=int((y==1).sum());falseground+=int((risk&(y==1)).sum());unsupported+=int((risk&~maximum_filter(vis,size=3,mode='constant')).sum());visible+=int(vis.sum());recalledvisible+=int((vis&maximum_filter(p>0,size=3,mode='constant')).sum())
  if risk.any():dist.extend((distance_transform_edt(~gt)*.1)[risk].tolist() if gt.any() else [99.]*int(risk.sum()))
  same=risk&gt;heightok+=int(((p==y)&same).sum());heightn+=int(same.sum())
 r=dict(predicted_risk_cells=n_pred,true_visible_risk_cells=n_gt,matched_predicted=matched_pred,matched_truth=matched_gt,precision=matched_pred/max(1,n_pred),recall=matched_gt/max(1,n_gt),ground_false_cells=falseground,ground_cells=ground,ground_false_fraction=falseground/max(1,ground),unsupported_predictions=unsupported,unsupported_fraction=unsupported/max(1,n_pred),visible_cells=visible,visible_surface_recall=recalledvisible/max(1,visible),localization_p90_m=float(np.quantile(dist,.9)) if dist else None,exact_overlap_height_accuracy=heightok/max(1,heightn),exact_overlap_height_cells=heightn)
 r['per_height']={}
 for k in [2,3,4]:
  npred=ngt=mp=mg=0
  for y,p in zip(Y,P):
   pr=p==k;gt=y==k;npred+=int(pr.sum());ngt+=int(gt.sum());mp+=int((pr&maximum_filter(gt,size=3,mode='constant')).sum());mg+=int((gt&maximum_filter(pr,size=3,mode='constant')).sum())
  r['per_height'][str(k)]=dict(predicted=npred,truth=ngt,precision=mp/max(1,npred),recall=mg/ngt if ngt else None,absent_truth=ngt==0)
 r['pass_gate']=bool(n_pred>0 and r['precision']>=.9 and r['recall']>=.8 and r['ground_false_fraction']<=.05 and r['unsupported_fraction']<=.1 and r['visible_surface_recall']>=.7 and r['localization_p90_m']<=.15);return r
rows=[]
for name in ['H','E0','E1']:
 y=np.load(D/'data'/f'{name}.npz')['labels'];p=np.load(D/'predictions'/f'{name}.npz')['labels'];row=dict(group=name,role='training' if name!='E1' else 'fit_excluded_development',model=metrics(y,p),all_unknown=metrics(y,np.zeros_like(y)),train_frequency_map=metrics(y,np.broadcast_to(prior,y.shape)));rows.append(row);print(row,flush=True)
result=dict(results=rows,retain_source=rows[-1]['model']['pass_gate'],fit=1,physics=0,limits='frame-cell instances, not independent trials; one-cell Chebyshev tolerance for precision/recall, Euclidean for localization; labels are finite visible surfaces not hidden volume/full real free space; E1 researcher-seen development only; ground prediction not free; no navigation');(D/'result.json').write_text(json.dumps(result,indent=2)+'\n')
