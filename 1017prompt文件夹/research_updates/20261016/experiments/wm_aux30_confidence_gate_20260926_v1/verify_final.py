from pathlib import Path
import json,hashlib,re,subprocess,time
import torch
R=Path(__file__).resolve().parent;read=lambda p:json.loads(Path(p).read_text());p=read(R/'protocol.json');ROOT=Path(p['root']);A=Path(p['frozen_acceleration_run'])
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
for f,h in p['input_sha256'].items():assert sha(f)==h,f
for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
for f,h in p['new_code_sha256'].items():assert sha(R/f)==h,f
oldp=read(A/'protocol.json');fingerprints=read(Path(oldp['baseline_run'])/'dataset_sha256.json')['files']
for f,h in fingerprints.items():assert sha(Path(oldp['data_root'])/f)==h,f
assert read(R/'status.json')['status']=='complete'
a=read(R/'assessment.json');assert len(a['checks'])==28 and a['passed']==24 and not a['all_passed']
assert sum(x['passed'] for x in a['checks'][:24])==21 and not read(A/'assessment.json')['all_passed']
ck=torch.load(R/'gate.pt',map_location='cpu',weights_only=False);assert ck['steps']==500 and ck['protocol']==p
hist=read(R/'calibration_history.json');assert len(hist)==500 and [x['step'] for x in hist]==list(range(1,501))
assert ck['gate']['linear.weight'].abs().sum()>0
c=torch.load(A/'feature_windows.pt',map_location='cpu',weights_only=False);pp=c['tensors']['proprio'];assert pp.dtype==torch.float32 and pp.device.type=='cpu'
from confidence_gate import state_features
f=state_features({'proprio':pp[c['partitions'][0]]})
assert torch.equal(ck['gate']['feature_mean'],f.mean(0)) and torch.equal(ck['gate']['feature_scale'],f.std(0,unbiased=False).clamp_min(1e-4))
for s in ['val','test']:
 d=read(R/f'{s}_prediction.json');assert d['gate_sha256']==sha(R/'gate.pt') and d['aggregate']['window_count']==94
 assert all(x['exact_cv_count']==0 for x in read(R/'gate_usage.json')[s]['overall'].values())
assert '7 failed' in (R/'tests_red.log').read_text() and '7 passed' in (R/'tests_green.log').read_text()
assert '1210 passed, 1 xfailed, 3 warnings in 17.28s' in (R/'regression_tests.log').read_text()
report=R/'world_model_aux30_confidence_gate_20260926.md';dest=ROOT/'research/ball_wm/runs'/R.name
links=re.findall(r'\]\((/[^)]+)\)',report.read_text())
for f in links:
 local=R/Path(f).relative_to(dest)
 if local.name!='final_verification.json':assert local.exists(),f
check=subprocess.run(['git','diff','--check'],cwd=ROOT,capture_output=True,text=True);assert check.returncode==0,check.stdout+check.stderr
out={'passed':True,'verified_at_epoch':time.time(),'frozen_source_files_unchanged':len(p['source_sha256']),'parent_artifact_files_unchanged':len(p['input_sha256']),'data_files_unchanged':len(fingerprints),'new_source_files_verified':len(p['new_code_sha256']),'actual_fixed_updates':500,'candidate_count':1,'gate_parameter_count':175,'checks_passed':24,'checks_total':28,'candidate_accepted':False,'new_tests_passed':7,'regression_tests_passed':1210,'regression_xfailed':1,'full_suite':'90s timeout 124, Numba cache permission failure then resolved via writable cache for selected regression suite','excluded_regression_file':'test_dynamic_lattice_planning.py','new_simulations':0,'new_navigation_runs':0,'new_collection_sequences':0,'report_links_checked':len(links),'git_diff_check_returncode':0,'report_sha256':sha(report),'artifacts_sha256':{f.name:sha(f) for f in R.iterdir() if f.is_file() and f.name!='final_verification.json'}}
(R/'final_verification.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='artifacts_sha256'},indent=2))
