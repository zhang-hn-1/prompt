"""Author hashes, saved counts/gates/chronology only; no science replay."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
if __name__=='__main__':
 p=json.loads((H/'nominal_support_protocol.json').read_text());a=json.loads((H/'nominal_support_assessment.json').read_text())
 for name in ['source_sha256','input_sha256']:
  for path,d in p[name].items():assert sha(path)==d,path
 assert p['sealed_utc']<a['started_utc']<a['arms'][0]['started_utc']<a['arms'][0]['completed_utc']<a['arms'][1]['started_utc']<a['arms'][1]['completed_utc']<a['completed_utc']
 assert a['planner_searches']==p['maximum_hybrid_searches']==2 and a['candidate_count']==1
 assert a['policy_act_calls']==a['new_physical_sequences']==a['model_fits']==0
 assert a['cpu_geometry_eligibility_pass']==all(a['gates'].values()) and not a['cpu_geometry_eligibility_pass']
 assert not a['gates']['candidate_path_no_more_one_primitive'] and not a['gates']['candidate_objective_no_more_one_primitive']
 assert a['tracker_drive_model_unchanged'] and a['no_future_actual_labels']
 assert not a['navigation_admitted'] and not a['physical_model_admitted'] and not a['whole_research_complete']
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),assessment_sha256=sha(H/'nominal_support_assessment.json'),exact_original_reference=True,searches=2,candidate_count=1,cpu_geometry_eligibility_pass=False,new_physical_sequences=0,no_science_replay=True,whole_research_complete=False,automation='ACTIVE')
 (H/'nominal_support_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
