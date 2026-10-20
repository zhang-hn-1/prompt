from pathlib import Path
import json,hashlib,ast,time,re,subprocess
import torch,numpy as np
from training.acceleration_residual import AccelerationResidualPredictor
from scripts.run_wm_motion_residual import evaluate
S=Path(__file__).resolve().parent;read=lambda f:json.loads(Path(f).read_text());p=read(S/'protocol.json');R=Path(p['run_dir']);ROOT=Path(p['root']);A=Path(p['original_run']);C=Path(p['expanded_data_run']);O=Path(p['frozen_rate_comparator'])
def sha(f):
 h=hashlib.sha256()
 with Path(f).open('rb') as handle:
  for b in iter(lambda:handle.read(1048576),b''):h.update(b)
 return h.hexdigest()
assert read(R/'status.json')['status']=='complete'
for f,h in p['input_sha256'].items():assert sha(f)==h,f
for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
for f,h in read(R/'execution.json')['new_files_sha256'].items():assert sha(R/f)==h,f
old=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);new=torch.load(C/'calibration_features.pt',map_location='cpu',weights_only=False)
oldfit={old['metadata'][i]['episode'] for i in old['partitions'][0]};newfit={m['episode'] for m in new['metadata']};held={old['metadata'][i]['episode'] for group in old['partitions'][1:] for i in group};assert not ((oldfit|newfit)&held) and not oldfit&newfit
node=next(n for n in ast.parse((C/'pipeline.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='historical_checks');exec(compile(ast.Module(body=[node],type_ignores=[]),'frozen_historical_checks','exec'),globals())
recomputed={};largest_metric_delta=0.
for arm in ['A_original','B_expanded']:
 sel=read(R/arm/'selection.json');h=read(R/arm/'training_history.json');assert len(h)==30 and [x['optimizer_steps'] for x in h]==list(range(924,27721,924));assert sel['selected']==min(h,key=lambda x:(x['val_motion_loss'],x['optimizer_steps']));assert not sel['test_used_for_selection'];assert set(sel['fit_episodes'])==(oldfit if arm=='A_original' else oldfit|newfit)
 assert sha(sel['checkpoint'])==sel['checkpoint_sha256'];saved=torch.load(sel['checkpoint'],map_location='cpu',weights_only=False);assert saved['protocol']==p
 if arm=='A_original':
  original=torch.load(read(A/'checkpoint_selection.json')['checkpoint'],map_location='cpu',weights_only=False)['model'];assert all(torch.equal(v,original[k]) for k,v in saved['model'].items())
 model=AccelerationResidualPredictor();model.load_state_dict(saved['model']);model.eval();reports={}
 for split,group in zip(['val','test'],old['partitions'][1:]):
  ids=[i for i in group if old['metadata'][i]['start_frame']%5==0];reports[split]=evaluate(model,old['tensors'],ids,old['metadata']);prior=read(R/arm/f'{split}_prediction.json')
  assert len(ids)==94
  for horizon in ['0.1','0.3','0.5']:
   for key in ['displacement_error_m_mean','yaw_abs_error_deg_mean']:
    delta=abs(reports[split]['aggregate']['per_horizon'][horizon]['model'][key]-prior['aggregate']['per_horizon'][horizon]['model'][key]);largest_metric_delta=max(largest_metric_delta,delta);assert delta<1e-6
  b={k:v[ids] for k,v in old['tensors'].items()}
  with torch.inference_mode():
   pred=model(b);assert torch.equal(pred,model({k:v for k,v in b.items() if k!='target'}));assert torch.equal(pred,model({**b,'target':torch.full_like(b['target'],float('nan'))}))
 checks=historical_checks(reports);record=read(R/arm/'assessment.json');assert [x['passed'] for x in checks]==[x['passed'] for x in record['checks']]
 assert [{k:v for k,v in x.items() if k not in ['ratio','passed','count','delta']} for x in record['checks']]==p['historical_check_templates'];recomputed[arm]=sum(x['passed'] for x in checks)
raw_count=0
for manifest,root in [(p['original_dataset_manifest'],p['original_data_root']),(p['expanded_dataset_manifest'],None)]:
 m=read(manifest);base=Path(root or m['root'])
 for f,h in m['files'].items():assert sha(base/f)==h,f
 raw_count+=len(m['files'])
fresh=read(p['fresh_manifest']);assert sha(p['fresh_manifest'])==p['fresh_manifest_sha256'];dis=read(R/'disposition.json')
if not dis['historical_passed']:assert all(not Path(j['output']).exists() for j in fresh['jobs'])
report=S/'world_model_aux30_data_expansion_20260927.md';assert report.exists()
for link in re.findall(r'\]\((/[^)]+)\)',report.read_text()):assert Path(link).exists() or (S/Path(link).name).exists() or Path(link).name=='final_verification.json',link
check=subprocess.run(['git','diff','--check'],cwd=ROOT,capture_output=True,text=True);assert check.returncode==0
assert '4 passed' in (R/'tests_green.log').read_text()
result={'passed':True,'verified_epoch':time.time(),'both_arms_updates':27720,'both_arms_validation_opportunities':30,'original_selected_weight_tensors_identical':True,'whole_sequence_fit_isolation':True,'scores_recomputed_cpu':recomputed,'max_cpu_gpu_aggregate_metric_abs_delta':largest_metric_delta,'target_delete_nan_invariance_windows':376,'raw_files_unchanged':raw_count,'frozen_sources_unchanged':len(p['source_sha256']),'fresh_run':dis['fresh_run'],'disposition':dis,'report_sha256':sha(report),'git_diff_check':0}
(S/'final_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
