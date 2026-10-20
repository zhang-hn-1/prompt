from pathlib import Path
import json,datetime,hashlib
import numpy as np
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'floor_adapter_fit_protocol.json').read_text());fit=json.loads((H/'floor_adapter_fit.json').read_text());seal=json.loads((H/'semantic_adapter_heldout_seal.json').read_text());m=json.loads((H/'floor_adapter_heldout_predictions/manifest.json').read_text());a=json.loads((H/'floor_adapter_heldout_audit.json').read_text());d=json.loads((H/'floor_adapter_causal_diagnostic_v2.json').read_text());checks=0
 for obj,fields in [(p,['source_sha256','training_input_sha256','checkpoint_sha256']),(seal,['original_artifact_sha256','rgb_only_prediction_input_sha256','known_pose_projection_input_sha256'])]:
  for field in fields:
   for path,value in obj[field].items():assert sha(path)==value,path;checks+=1
 assert sha(H/'semantic_adapter_heldout_seal.json')==p['heldout_seal_sha256'];assert sha(fit['parameters'])==fit['parameters_sha256'];checks+=2
 assert seal['utc']<p['presealed_utc']<fit['started_utc']<fit['ended_utc']<m['rows'][0]['started_utc'];assert m['rows'][-1]['ended_utc']<a['started_utc']<a['ended_utc']<d['started_utc'];checks+=2
 expected={(i,e) for i in range(0,321,5) for e in ['left','right']}
 for rows in [m['rows'],a['rows'],d['rows']]:
  kk=[(r['frame'],r['eye']) for r in rows];assert len(kk)==len(set(kk))==130 and set(kk)==expected;checks+=1
 for r in m['rows']:assert sha(r['prediction'])==r['sha256'];checks+=1
 for r in fit['rows']:assert sha(r['file'])==r['sha256'];checks+=1
 assert len(fit['rows'])==24 and fit['samples']==51137 and fit['positives']==3591;assert fit['optimizer_converged'] and fit['optimizer_iterations']<=200;assert not a['candidate_accepted'];checks+=3
 ss=a['summary']; rr=a['rows']
 for field,rowfield in [('total_queries','queries'),('total_outputs','outputs'),('correct_outputs','correct_outputs'),('correct_queries','correct_queries')]:assert ss[field]==sum(r[rowfield] for r in rr);checks+=1
 assert ss['output_precision']==ss['correct_outputs']/ss['total_outputs'];assert ss['all_query_correct_fraction']==ss['correct_queries']/ss['total_queries'];assert ss['accepted_image_samples']==sum(r['accepted'] for r in rr)==0;checks+=3
 ds=d['summary'];assert ds['queries']==ss['total_queries'] and ds['actual_correct_queries']==ss['correct_queries'];assert ds['queries']==ds['lost_plane_geometry_or_ray']+ds['lost_only_mask_footprint']+ds['oracle_correct_ceiling'];assert ds['oracle_correct_ceiling']-ds['classifier_misses_within_ceiling']==ss['correct_queries'];assert ds['false_outputs']==ss['total_outputs']-ss['correct_outputs'];checks+=4
 for v in d['videos']:assert v['decoded_frames']==321 and v['fps']==10 and sha(v['file'])==v['sha256'];checks+=1
 assert '299 passed in 10.27s' in (H/'floor_adapter_complete_suite.log').read_text();assert '\nstatus = "ACTIVE"' in Path('/home/an/.codex/automations/automation/automation.toml').read_text();checks+=2
 (H/'floor_adapter_author_verify.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),passed=True,checks=checks,candidate_rejected=True,whole_research_complete=False,new_fits=0,new_prediction=0,new_physics=0,automation='ACTIVE'),indent=2)+'\n');print('ADAPTER_AUTHOR_VERIFY_PASS',checks)
if __name__=='__main__':main()
