from pathlib import Path
import json,hashlib,datetime
from suffix_completion_semantics_v1 import suffix_completion_eligibility
H=Path(__file__).resolve().parent;D=H/'forward_outer_yaw_e1_b0_run_v1'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
if __name__=='__main__':
 seal=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(Path(p))==h for p,h in seal['artifact_sha256'].items());status=json.loads((H/'forward_outer_yaw_e1_b0_status_v1.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
 rows=json.loads((D/'policy_records.json').read_text());result=[]
 for i,current in enumerate(rows):
  d=current.get('reference_decision',{})
  if not d or abs(float(d.get('measured_time_s',-1))-current['time_s'])>1e-6:continue
  if i==0 or not rows[i-1].get('planner_act_called'):
   result.append(dict(frame=current['frame'],eligible=False,reasons=['no_past_installed_plan']));continue
  previous=rows[i-1];file=D/'local_plans'/f'frame_{previous["frame"]:03d}.json';plan=json.loads(file.read_text()) if file.exists() else dict(frame=-1,reference_version=-1)
  result.append(dict(frame=current['frame'],time_s=current['time_s'],**suffix_completion_eligibility(current,previous,plan)))
 inputs=[D/'policy_records.json',D/'artifact_manifest_post_close.json',H/'forward_outer_yaw_e1_b0_status_v1.json',*sorted((D/'local_plans').glob('*.json'))];sources=[H/x for x in ['suffix_completion_semantics_scope_v1.json','suffix_completion_semantics_v1.py','test_suffix_completion_semantics_v1.py','audit_suffix_completion_semantics_v1.py']]
 out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='one sealed current B0 sequence; no solver or physical run',seal_items_verified=len(seal['artifact_sha256']),input_sha256={str(p):sha(p) for p in inputs},source_sha256={str(p):sha(p) for p in sources},boundary_count=len(result),eligible_frames=[r['frame'] for r in result if r['eligible']],rows=result,current_past_only=True,future_counterfactual_labels_available=False,physical_budget_used=0,CPU_solver_queries=0,whole_mission_complete=False)
 p=H/'suffix_completion_semantics_assessment_v2.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ['source_sha256','input_sha256','rows']}))
