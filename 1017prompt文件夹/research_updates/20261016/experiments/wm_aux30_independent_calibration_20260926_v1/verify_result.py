from pathlib import Path
import json,hashlib,time,re,subprocess
import torch
STAGE=Path(__file__).resolve().parent
R=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1')
read=lambda p:json.loads(Path(p).read_text());p=read(R/'protocol.json');ROOT=Path(p['root'])
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
assert read(R/'resume_status.json')['status']=='complete'
for f,h in p['input_sha256'].items():assert sha(f)==h,f
for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
for f,h in read(STAGE/'additional_dependency_fingerprints.json')['files'].items():assert sha(f)==h,f
assert sha(R/'pipeline.py')==read(R/'execution.json')['pipeline_sha256']==read(R/'preflight.json')['pipeline_sha256']
assert sha(R/'confidence_gate.py')==p['gate_source_sha256']==sha(Path(p['parent_gate_run'])/'confidence_gate.py')
amendment=read(R/'calibration_coverage_amendment.json')
for f,h in amendment['preserved_original_artifacts'].items():assert sha(R/f)==h,f
assert sha(R/'resume_pipeline.py')==amendment['new_runner_sha256']
assert sha(R/'calibration_eligibility.py')==amendment['eligibility_source_sha256']
assert read(R/'status.json')['status']=='failed'
from calibration_eligibility import check_eligibility
records=read(R/'resume_collection_summary.json');qcs=[]
for row in records:
 report=read(R/('resume_schema_'+row['id']+'.json'));eligible=check_eligibility(row['id'],row['group'],row['sequence_sha256'],report['errors'])
 assert all(row[k]==v for k,v in eligible.items())
 assert row['returncode']==row['schema_audit_returncode']==int(eligible['coverage_exception']) and row['quality_passed'] and row['finalized'];q=read(R/('resume_qc_'+row['id']+'.json'));assert q['passed'] and all(q['checks'].values());qcs.append(q)
fingerprints=read(R/'calibration_dataset_sha256.json')
for f,h in fingerprints['files'].items():assert sha(Path(fingerprints['root'])/f)==h,f
cal=torch.load(R/'calibration_features.pt',map_location='cpu',weights_only=False);assert len(cal['metadata'])==924 and len({m['episode'] for m in cal['metadata']})==4
old=torch.load(Path(p['frozen_acceleration_run'])/'feature_windows.pt',map_location='cpu',weights_only=False);assert not {m['episode'] for m in cal['metadata']} & {m['episode'] for m in old['metadata']}
from confidence_gate import state_features,StateConfidenceGate,GatedPredictor
from training.acceleration_residual import AccelerationResidualPredictor
ck=torch.load(R/'gate.pt',map_location='cpu',weights_only=False);assert ck['steps']==500 and ck['protocol']==p
f=state_features(cal['tensors']);assert torch.equal(ck['gate']['feature_mean'],f.mean(0)) and torch.equal(ck['gate']['feature_scale'],f.std(0,unbiased=False).clamp_min(1e-4))
h=read(R/'calibration_history.json');assert len(h)==500 and [x['step'] for x in h]==list(range(1,501));assert ck['gate']['linear.weight'].abs().sum()>0
assessment=read(R/'assessment.json');assert len(assessment['checks'])==28 and assessment['passed']==sum(x['passed'] for x in assessment['checks'])
templates=[{k:v for k,v in x.items() if k not in ['ratio','delta','count','passed']} for x in assessment['checks']];assert templates==p['historical_check_templates']
for s in ['val','test']:
 d=read(R/f'{s}_prediction.json');assert d['aggregate']['window_count']==94 and d['gate_sha256']==sha(R/'gate.pt')
 assert d['aggregate']['action_diagnostics']['prefix_pose_max_abs_delta']<=1e-5
fresh=read(p['fresh_manifest']);assert sha(p['fresh_manifest'])==p['fresh_manifest_sha256']
disposition=read(R/'disposition.json')
if not assessment['all_passed']:
 assert not disposition['fresh_run'];assert all(not Path(j['output']).exists() for j in fresh['jobs']);assert len(records)==4
else:
 assert len(records)==8 and (R/'fresh_assessment.json').exists()
 for path,hash_value in read(R/'fresh_dataset_sha256.json')['files'].items():assert sha(Path(read(R/'fresh_dataset_sha256.json')['root'])/path)==hash_value
assert '7 passed' in (R/'preflight_tests.log').read_text()
assert '7 passed' in (R/'resume_preflight_tests.log').read_text()
assert '2 passed' in (STAGE/'original_coverage_tests.log').read_text()
review=read(STAGE/'independent_postfit_review.json');assert review['28_scores_recomputed_match'] and review['checks_passed']==16 and review['normalization_exact_new_calibration_only']
assert review['gate_sha256']==sha(R/'gate.pt')
report=STAGE/'world_model_aux30_independent_calibration_20260926.md';links=re.findall(r'\]\((/[^)]+)\)',report.read_text())
for target in links:
 path=Path(target)
 if path.exists():continue
 if path.parent==R and (STAGE/path.name).exists():continue
 if path.name=='final_verification.json':continue
 raise AssertionError(target)
check=subprocess.run(['git','diff','--check'],cwd=ROOT,capture_output=True,text=True);assert check.returncode==0,check.stdout+check.stderr
summary={'passed':True,'verified_at_epoch':time.time(),'new_calibration_sequences':4,'calibration_windows':924,'calibration_raw_files_unchanged':len(fingerprints['files']),'input_artifact_files_unchanged':len(p['input_sha256']),'frozen_sources_unchanged':len(p['source_sha256']),'qc_checks':sum(len(q['checks']) for q in qcs),'quality_passed_sequences':len(qcs),'original_formal_schema_passes':sum(row['formal_schema_passed'] for row in records),'calibration_only_coverage_exceptions':sum(row['coverage_exception'] for row in records),'original_failed_attempt_preserved':True,'resume_preflight_tests_passed':7,'original_coverage_tests_passed':2,'independent_full_score_recomputation':True,'fixed_gate_updates':500,'historical_passed':assessment['passed'],'historical_total':28,'disposition':disposition,'preflight_tests_passed':7,'git_diff_check_returncode':check.returncode,'report_links_checked':len(links),'report_sha256':sha(report),'new_artifacts_sha256':{x.name:sha(x) for x in STAGE.iterdir() if x.is_file() and x.name!='final_verification.json'}}
(STAGE/'final_verification.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='new_artifacts_sha256'},indent=2))
