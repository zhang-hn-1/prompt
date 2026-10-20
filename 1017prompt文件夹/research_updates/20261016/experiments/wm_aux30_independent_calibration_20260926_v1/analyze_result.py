"""Post-fit reporting diagnostics. Never used for candidate selection."""
from pathlib import Path
import json,sys
import torch,numpy as np
STAGE=Path(__file__).resolve().parent;R=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1')
read=lambda p:json.loads(Path(p).read_text());p=read(R/'protocol.json');A=Path(p['frozen_acceleration_run']);G=Path(p['parent_gate_run'])
from confidence_gate import StateConfidenceGate,GatedPredictor,state_features,scale_targets
from training.acceleration_residual import AccelerationResidualPredictor
assert read(R/'resume_status.json')['status']=='complete'
base=AccelerationResidualPredictor();base.load_state_dict(torch.load(read(A/'checkpoint_selection.json')['checkpoint'],map_location='cpu',weights_only=False)['model']);base.eval()
saved=torch.load(R/'gate.pt',map_location='cpu',weights_only=False);state=saved['gate'];gate=StateConfidenceGate(state['feature_mean'],state['feature_scale']);gate.load_state_dict(state);gate.eval()
cache=torch.load(R/'calibration_features.pt',map_location='cpu',weights_only=False)
assert len(cache['metadata'])==924
old=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False)
sets={'calibration_fit':cache}
for s,ids in zip(['val','test'],old['partitions'][1:]):sets[s]={'tensors':{k:v[ids] for k,v in old['tensors'].items()},'metadata':[old['metadata'][i] for i in ids]}
if (R/'fresh_features.pt').exists():sets['fresh']=torch.load(R/'fresh_features.pt',map_location='cpu',weights_only=False)
result={}
for s,c in sets.items():
 ids=[i for i,m in enumerate(c['metadata']) if m['start_frame']%5==0];b={k:v[ids] for k,v in c['tensors'].items()}
 with torch.inference_mode():pred=base(b);target,energy=scale_targets(pred,b['prior'],b['target']);raw=gate.coefficients(state_features(b),apply_cutoff=False);used=gate(b)
 result[s]={}
 for step in [0,2,4]:
  r=raw[:,step];a=target[:,step];w=energy[:,step];cxy=pred[:,step,:2]-b['prior'][:,step,:2];rxy=b['target'][:,step,:2]-b['prior'][:,step,:2];bad=(cxy*rxy).sum(-1)<0;fallback=used[:,step]==0
  base_error=(pred[:,step,:2]-b['target'][:,step,:2]).norm(dim=-1);cv_error=(b['prior'][:,step,:2]-b['target'][:,step,:2]).norm(dim=-1)
  gated_xy=b['prior'][:,step,:2]+used[:,step,None]*cxy;gated_error=(gated_xy-b['target'][:,step,:2]).norm(dim=-1)
  result[s][f'{(step+1)*.1:.1f}']={'fallback_while_base_better_count':int((fallback & (base_error<cv_error)).sum()),'fallback_while_base_worse_count':int((fallback & (base_error>cv_error)).sum()),'kept_base_worse_count':int((~fallback & (base_error>cv_error)).sum()),'gate_worse_than_base_count':int((gated_error>base_error).sum()),'gate_better_than_base_count':int((gated_error<base_error).sum()),'count':len(ids),'optimal_scale_zero_count':int((a==0).sum()),'mean_optimal_scale':float(a.mean()),'energy_weighted_optimal_scale':float((a*w).sum()/w.sum()),'mean_raw_coefficient':float(r.mean()),'min_raw_coefficient':float(r.min()),'max_raw_coefficient':float(r.max()),'exact_fallback_count':int(fallback.sum()),'xy_wrong_direction_count':int(bad.sum()),'xy_wrong_direction_retained':int((bad & ~fallback).sum()),'xy_wrong_direction_removed':int((bad & fallback).sum())}
(STAGE/'postfit_diagnostics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
