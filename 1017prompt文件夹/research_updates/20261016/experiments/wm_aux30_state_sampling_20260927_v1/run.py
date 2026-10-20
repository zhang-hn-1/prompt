from pathlib import Path
import json,hashlib,time,random,traceback,ast
import numpy as np
import torch
from training_scope import expanded_partition
from training.acceleration_residual import AccelerationResidualPredictor
from training.motion_residual import motion_loss
from scripts.run_wm_motion_residual import evaluate
R=Path(__file__).resolve().parent;read=lambda f:json.loads(Path(f).read_text());p=read(R/'protocol.json');ROOT=Path(p['root']);A=Path(p['original_run']);C=Path(p['expanded_data_run']);O=Path(p['frozen_rate_comparator']);B=Path(p['baseline_expanded_run'])
def sha(f):
 h=hashlib.sha256()
 with Path(f).open('rb') as handle:
  for chunk in iter(lambda:handle.read(1048576),b''):h.update(chunk)
 return h.hexdigest()
def dump(name,x):(R/name).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def integrity():
 for f,h in p['input_sha256'].items():assert sha(f)==h,f
 for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
node=next(n for n in ast.parse((C/'pipeline.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='historical_checks');exec(compile(ast.Module(body=[node],type_ignores=[]),'frozen_checks','exec'),globals())
exec((R/'frozen_train_function.py').read_text(),globals())
assert not (R/'status.json').exists();status={'status':'running','started_epoch':time.time(),'phase':'integrity'};dump('status.json',status)
try:
 integrity();assert torch.cuda.is_available();dump('runtime.json',{'torch':torch.__version__,'cuda':torch.version.cuda,'gpu':torch.cuda.get_device_name(0)})
 old=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);extra=torch.load(C/'calibration_features.pt',map_location='cpu',weights_only=False);assert old['feature_checkpoint_sha256']==extra['feature_checkpoint_sha256']==p['original_feature_sha256'];parts=expanded_partition(old['metadata'],old['partitions'],extra['metadata']);cached={'tensors':{k:torch.cat([v,extra['tensors'][k]]) for k,v in old['tensors'].items()},'metadata':old['metadata']+extra['metadata']}
 assert sha(R/'schedule.npy')==p['sampling']['schedule_sha256'];assert sha(R/'sampling_scope.json')==p['sampling']['scope_sha256'];schedule=np.load(R/'schedule.npy');assert schedule.shape==(27720,2) and set(schedule.ravel())==set(parts[0])
 initial=torch.load(A/'initial_weights.pt',map_location='cpu',weights_only=False);status['phase']='train_C_state_balanced';dump('status.json',status);history,reports=train_arm('C_balanced',cached,parts,None,initial)
 checks=read(R/'C_balanced/assessment.json')['checks']
 for split in ['val','test']:
  previous=read(B/f'{split}_prediction.json')
  for h in ['0.1','0.5']:
   ratio=reports[split]['aggregate']['per_horizon'][h]['model']['displacement_error_m_mean']/previous['aggregate']['per_horizon'][h]['model']['displacement_error_m_mean'];checks.append({'split':split,'check':'preserve_B_'+h+'_xy','ratio':ratio,'limit':1.05,'passed':ratio<=1.05})
 selected=read(R/'C_balanced/selection.json')['selected']['optimizer_steps'];groups=read(R/'sampling_scope.json')['groups'];mapping={int(i):g for g,ids in groups.items() for i in ids};counts={g:sum(mapping[int(i)]==g for i in schedule[:selected].ravel()) for g in groups};dump('selected_exposure.json',{'selected_updates':selected,'window_presentations':selected*2,'group_exposures':counts})
 passed=all(x['passed'] for x in checks);dump('assessment.json',{'checks':checks,'passed':sum(x['passed'] for x in checks),'total':32,'all_passed':passed,'original28_passed':sum(x['passed'] for x in checks[:28])})
 integrity();assert all(not Path(j['output']).exists() for j in read(p['fresh_manifest'])['jobs']);dump('disposition.json',{'historical_passed':passed,'fresh_required':passed,'fresh_run':False,'candidate_accepted':False,'navigation_runs':0,'reason':'Fresh required' if passed else 'Historical failed; candidate rejected'})
 status.update(status='complete',phase='historical_complete',ended_epoch=time.time());dump('status.json',status)
except BaseException as e:
 status.update(status='failed',error=f'{type(e).__name__}: {e}',ended_epoch=time.time());dump('status.json',status);traceback.print_exc();raise
