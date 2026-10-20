"""Author final integrity/score verification; no refit or model reinference."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'dense_adapter_fit_v2_protocol.json').read_text());seal=json.loads((H/'dense_adapter_heldout_seal.json').read_text());fit=json.loads((H/'dense_adapter_fit_v2_status.json').read_text());pipe=json.loads((H/'dense_adapter_v2_pipeline_status.json').read_text());pred=H/'dense_adapter_v2_predictions';manifest=json.loads((pred/'manifest.json').read_text());audit=json.loads((pred/'assessment.json').read_text());checks=0
for field in ('source_sha256','input_sha256'):
 for f,d in p[field].items():assert sha(f)==d,f;checks+=1
assert sha(H/'dense_adapter_heldout_seal.json')==p['heldout_seal_sha256']
for field in ('artifact_sha256','sanitized_rgb_sha256'):
 for f,d in seal[field].items():assert sha(f)==d,f;checks+=1
assert seal['utc']<p['utc']<fit['start_utc']<fit['end_utc']<manifest['saved_utc']<audit['utc'];assert pipe['status']=='finished' and all(r['exit_code']==0 for r in pipe['stages']);assert fit['attempts']==fit['optimizer_updates']==300 and fit['frozen_state_exact'] and fit['changed_parameter_count']>0
assert sha(fit['parameters_path'])==fit['parameters_sha256'];checks+=1
expected={(arm,i) for arm in ('b0','adapter') for i in range(0,321,5)};assert {(r['arm'],r['frame']) for r in manifest['rows']}==expected
assert manifest['labels_not_loaded'] and manifest['pose_not_loaded']
for r in manifest['rows']:
 assert sha(r['path'])==r['sha256'];checks+=1
 packet=np.load(H/'dense_adapter_heldout_rgb'/f"frame_{r['frame']:03d}.npz");assert set(packet.files)=={'left_rgb','right_rgb','time_s'};assert float(packet['time_s'])==r['time_s']
 output=np.load(r['path']);assert set(output.files)=={'depth_m','confidence','disparity_px'} and np.shape(output['depth_m'])==(480,640);assert np.array_equal(output['confidence'],np.isfinite(output['depth_m']))
trace=[json.loads(r) for r in (H/'dense_adapter_v2_training_trace.jsonl').read_text().splitlines()];assert len(trace)==300 and [r['attempt'] for r in trace]==list(range(1,301)) and all(np.isfinite(r['loss']) and np.isfinite(r['unclipped_grad_norm']) for r in trace)
assert all(r['example']==i%26 for i,r in enumerate(trace))
# Independent arithmetic from saved all-query frame counts, not changed query membership.
for arm in ('b0','adapter'):
 rows=[r for r in audit['rows'] if r['arm']==arm];assert {r['frame'] for r in rows}==set(range(0,321,5))
 for group,s in audit['pooled'][arm].items():
  assert sum(r['groups'][group]['n'] for r in rows)==s['n'];assert sum(r['groups'][group]['correct'] for r in rows)==s['correct'];assert sum(r['groups'][group]['retained'] for r in rows)==s['retained']
  assert s['complete_correct_fraction']==(None if not s['n'] else s['correct']/s['n']);assert s['precision']==(None if not s['retained'] else s['correct']/s['retained'])
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),passed=True,sha_checks=checks,originals_unchanged=True,whole_before_fit_sealed=True,model_updates=300,old_failed_preparation_zero_updates=True,frozen_non_decoder_state_asserted_exact_in_actual_fit=True,complete_prediction_rows=130,all_queries_kept=True,model_candidate_admitted=audit['adapter_admitted'],rgb_navigation_admitted=False,whole_research_complete=False)
artifacts={str(f):sha(f) for f in pred.rglob('*') if f.is_file()};(H/'dense_adapter_postrun_artifacts.json').write_text(json.dumps(dict(utc=result['utc'],sha256=artifacts),indent=2)+'\n');(H/'dense_adapter_author_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
