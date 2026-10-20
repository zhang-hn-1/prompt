"""After-the-fact audit only; no fitting or threshold selection."""
from pathlib import Path
import json,torch,numpy as np
from confidence_gate import scale_targets,state_features,StateConfidenceGate
from training.acceleration_residual import AccelerationResidualPredictor
R=Path(__file__).resolve().parent;read=lambda p:json.loads(Path(p).read_text());p=read(R/'protocol.json');A=Path(p['frozen_acceleration_run'])
c=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);t=c['tensors'];meta=c['metadata'];base=AccelerationResidualPredictor();base.load_state_dict(torch.load(read(A/'checkpoint_selection.json')['checkpoint'],map_location='cpu',weights_only=False)['model']);base.eval()
saved=torch.load(R/'gate.pt',map_location='cpu',weights_only=False);state=saved['gate'];gate=StateConfidenceGate(state['feature_mean'],state['feature_scale']);gate.load_state_dict(state);gate.eval()
result={}
for split,ids in zip(['train','val','test'],c['partitions']):
 ids=[i for i in ids if meta[i]['start_frame']%5==0];b={k:v[ids] for k,v in t.items()}
 with torch.inference_mode():pred=base(b);alpha,energy=scale_targets(pred,b['prior'],b['target']);raw=gate.coefficients(state_features(b),apply_cutoff=False);applied=gate(b)
 result[split]={}
 for step in [0,2,4]:
  a=alpha[:,step];e=energy[:,step];r=raw[:,step]
  cxy=pred[:,step,:2]-b['prior'][:,step,:2];rxy=b['target'][:,step,:2]-b['prior'][:,step,:2]
  result[split][str((step+1)*.1)]={'count':len(ids),'training_style_optimal_scale_mean':float(a.mean()),'energy_weighted_optimal_scale':float((e*a).sum()/e.sum()),'zero_optimal_scale_count':int((a==0).sum()),'raw_coefficient_mean':float(r.mean()),'raw_coefficient_min':float(r.min()),'raw_coefficient_max':float(r.max()),'coefficient_below_half_count':int((r<.5).sum()),'coefficient_weighted_squared_error':float((e*(r-a).square()).sum()/e.sum()),'xy_opposite_direction_count':int(((cxy*rxy).sum(-1)<0).sum()),'xy_opposite_direction_but_retained_count':int((((cxy*rxy).sum(-1)<0)&(applied[:,step]>0)).sum())}
(R/'calibration_diagnostics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
