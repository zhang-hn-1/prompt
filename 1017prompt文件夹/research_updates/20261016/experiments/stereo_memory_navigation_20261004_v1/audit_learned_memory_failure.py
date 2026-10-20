"""One bounded retrospective error-layer diagnosis, no gate/model edits."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
from scipy.ndimage import distance_transform_edt
from cell_proxy_metrics import confusion
H=Path(__file__).resolve().parent;P=H/'learned_memory_held_predictions';A=H/'learned_memory_held_assessment/assessment.json';assert not json.loads(A.read_text())['model_validation_passed'];fit=np.load(H/'learned_memory_fit/fit_predictions.npz');result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='fit score versus complete fresh validation; fixed .10m grid-boundary diagnostic only; no gate rescue',fit_descriptive=confusion(fit['scores']>=.5,fit['labels']>0),cases=[])
for case in ['memory_validation_far_run','memory_validation_near_run']:
 occ=np.load(H/case/'offline_occupancy_label.npz');dist=distance_transform_edt(occ['labels']<=0)*float(occ['resolution']);fpd=[];basefpd=[];bins=[]
 for p in sorted((P/case).glob('*.npz')):
  d=np.load(p);xy=d['origin']+d['cells']*float(d['resolution']);ij=np.rint((xy-occ['origin'])/float(occ['resolution'])).astype(int);valid=(ij>=0).all(1)&(ij<occ['labels'].shape).all(1);ij=ij[valid];y=occ['labels'][ij[:,0],ij[:,1]]>0;a=d['scores'][valid]>=.5;b=d['baseline_history_labels'][valid]==1;fpd.extend(dist[ij[a&~y,0],ij[a&~y,1]].tolist());basefpd.extend(dist[ij[b&~y,0],ij[b&~y,1]].tolist());bins.append(dict(time_s=float(d['time_s']),model=confusion(a,y),baseline=confusion(b,y)))
 result['cases'].append(dict(case=case,model_FP=len(fpd),model_FP_within_10cm_of_occupied_cell=int((np.array(fpd)<=.100000001).sum()),model_FP_nearest_occupied_distance_p50_p90=None if not fpd else np.percentile(fpd,[50,90]).tolist(),baseline_FP=len(basefpd),baseline_FP_within_10cm=int((np.array(basefpd)<=.100000001).sum()),temporal_bins=bins,not_a_modified_acceptance_gate=True))
(H/'learned_memory_failure_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({**result,'cases':[{k:v for k,v in c.items() if k!='temporal_bins'} for c in result['cases']]},indent=2))
