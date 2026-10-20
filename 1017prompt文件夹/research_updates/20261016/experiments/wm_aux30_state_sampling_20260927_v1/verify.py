from pathlib import Path
import json,hashlib,ast,time,re,subprocess
import torch,numpy as np
from sampling import balanced_schedule,classify_anchor
from training_scope import expanded_partition
from training.acceleration_residual import AccelerationResidualPredictor
from scripts.run_wm_motion_residual import evaluate
S=Path(__file__).resolve().parent;read=lambda f:json.loads(Path(f).read_text());p=read(S/'protocol.json');R=Path(p['run_dir']);ROOT=Path(p['root']);A=Path(p['original_run']);C=Path(p['expanded_data_run']);O=Path(p['frozen_rate_comparator']);B=Path(p['baseline_expanded_run'])
def sha(f):
 h=hashlib.sha256()
 with Path(f).open('rb') as handle:
  for v in iter(lambda:handle.read(1048576),b''):h.update(v)
 return h.hexdigest()
assert read(R/'status.json')['status']=='complete'
for f,h in p['input_sha256'].items():assert sha(f)==h,f
for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
for f,h in read(R/'execution.json')['frozen_artifact_sha256'].items():assert sha(R/f)==h,f
source=(B.parent/'run_ab.py').read_text();tree=ast.parse(source);old_function=ast.get_source_segment(source,next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='train_arm'));expected=old_function.replace("batch_schedule(parts[0],epochs,2,p['seed'])",'schedule.tolist()')+'\n';assert expected==(R/'frozen_train_function.py').read_text()
old=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);extra=torch.load(C/'calibration_features.pt',map_location='cpu',weights_only=False);parts=expanded_partition(old['metadata'],old['partitions'],extra['metadata']);scope=read(R/'sampling_scope.json');groups={int(k):v for k,v in scope['groups'].items()};schedule=np.load(R/'schedule.npy');assert np.array_equal(schedule,balanced_schedule(groups,27720,2,3072));assert set(schedule.ravel())==set(parts[0])
assert all(classify_anchor(r['xy_speed'],r['forward_speed'],r['aux_deg'])==r['group'] for r in scope['anchors']);mapping={i:g for g,ids in groups.items() for i in ids};full=np.bincount([mapping[int(i)] for i in schedule.ravel()]);assert full.tolist()==[49896,2772,2772]
h=read(R/'C_balanced/training_history.json');sel=read(R/'C_balanced/selection.json');assert len(h)==30 and [x['optimizer_steps'] for x in h]==list(range(924,27721,924));assert sel['selected']==min(h,key=lambda x:(x['val_motion_loss'],x['optimizer_steps']));assert not sel['test_used_for_selection'];assert sha(sel['checkpoint'])==sel['checkpoint_sha256']
count=np.bincount([mapping[int(i)] for i in schedule[:sel['selected']['optimizer_steps']].ravel()]);assert {str(i):int(n) for i,n in enumerate(count)}==read(R/'selected_exposure.json')['group_exposures']
fit_episodes={old['metadata'][i]['episode'] for i in old['partitions'][0]}|{r['episode'] for r in extra['metadata']};assert set(sel['fit_episodes'])==fit_episodes;assert not fit_episodes&{old['metadata'][i]['episode'] for g in old['partitions'][1:] for i in g}
node=next(n for n in ast.parse((C/'pipeline.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='historical_checks');exec(compile(ast.Module(body=[node],type_ignores=[]),'frozen_historical_checks','exec'),globals())
model=AccelerationResidualPredictor();model.load_state_dict(torch.load(sel['checkpoint'],map_location='cpu',weights_only=False)['model']);model.eval();reports={};maxdelta=0.
for split,group in zip(['val','test'],old['partitions'][1:]):
 ids=[i for i in group if old['metadata'][i]['start_frame']%5==0];assert len(ids)==94;reports[split]=evaluate(model,old['tensors'],ids,old['metadata']);original=read(R/'C_balanced'/f'{split}_prediction.json')
 for horizon in ['0.1','0.3','0.5']:
  for key in ['displacement_error_m_mean','yaw_abs_error_deg_mean']:
   delta=abs(reports[split]['aggregate']['per_horizon'][horizon]['model'][key]-original['aggregate']['per_horizon'][horizon]['model'][key]);maxdelta=max(delta,maxdelta);assert delta<1e-6
 with torch.inference_mode():
  b={k:v[ids] for k,v in old['tensors'].items()};pred=model(b);assert torch.equal(pred,model({k:v for k,v in b.items() if k!='target'}));assert torch.equal(pred,model({**b,'target':torch.full_like(b['target'],float('nan'))}))
checks=historical_checks(reports)
for split in ['val','test']:
 for h in ['0.1','0.5']:
  ratio=reports[split]['aggregate']['per_horizon'][h]['model']['displacement_error_m_mean']/read(B/f'{split}_prediction.json')['aggregate']['per_horizon'][h]['model']['displacement_error_m_mean'];checks.append({'passed':ratio<=1.05})
assessment=read(R/'assessment.json');assert len(checks)==32 and [x['passed'] for x in checks]==[x['passed'] for x in assessment['checks']]
assert [{k:v for k,v in x.items() if k not in ['ratio','delta','count','passed']} for x in assessment['checks'][:28]]==p['historical_check_templates']
assert all(not Path(j['output']).exists() for j in read(p['fresh_manifest'])['jobs'])
# State-group diagnostics use frozen previously recorded measured-state membership only, never refit.
f=ROOT/'research/ball_wm/runs/wm_aux30_error_localization_20260927_v1/rows.json';membership={(r['episode'],r['start_frame']):r['groups'] for r in read(f) if r['primary']};group_report={}
for split in ['val','test']:
 group_report[split]={}
 for label,criterion in [('reverse',lambda g:g['motion']=='reverse'),('reverse_fast_large',lambda g:g['motion']=='reverse' and g['speed']=='fast' and g['aux']=='large')]:
  rows=[r for r in reports[split]['rows'] if criterion(membership[(r['episode'],r['start_frame'])])];prior=[r for r in read(B/f'{split}_prediction.json')['rows'] if criterion(membership[(r['episode'],r['start_frame'])])];assert len(rows)==len(prior)
  group_report[split][label]={'count':len(rows),'B_yaw_deg':float(np.mean([np.rad2deg(r['per_horizon']['0.5']['model']['yaw_abs_error_rad']) for r in prior])) if rows else None,'C_yaw_deg':float(np.mean([np.rad2deg(r['per_horizon']['0.5']['model']['yaw_abs_error_rad']) for r in rows])) if rows else None}
(S/'state_group_diagnostics.json').write_text(json.dumps({'membership_source':str(f),'membership_sha256':sha(f),'groups':group_report},indent=2)+'\n')
assert '20 passed' in (S/'behavior_tests.log').read_text();assert '6 passed' in (R/'tests_green.log').read_text()
report=S/'world_model_aux30_state_sampling_20260927.md'
for link in re.findall(r'\]\((/[^)]+)\)',report.read_text()):assert Path(link).exists() or (S/Path(link).name).exists() or Path(link).name=='final_verification.json'
result={'passed':True,'verified_epoch':time.time(),'training_function_only_iterator_changed':True,'schedule_reproduced':True,'full_group_presentations':full.tolist(),'selected_group_presentations':count.tolist(),'whole_sequence_fit_isolation':True,'fixed_updates':27720,'fixed_validation_points':30,'recomputed_original28_passed':sum(x['passed'] for x in checks[:28]),'recomputed_total32_passed':sum(x['passed'] for x in checks),'max_cpu_gpu_aggregate_delta':maxdelta,'target_delete_NaN_invariance_windows':188,'inputs_unchanged':len(p['input_sha256']),'sources_unchanged':len(p['source_sha256']),'behavior_tests_passed':20,'fresh_untouched':True,'report_sha256':sha(report),'state_groups':group_report}
check=subprocess.run(['git','diff','--check'],cwd=ROOT,capture_output=True,text=True);assert check.returncode==0
(S/'final_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
