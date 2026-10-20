"""Post-seal intervention identity, no alternate trajectory or effect attribution."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'original_E1_repeated_exposure_memory_navigation_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f
rows=json.loads((D/'policy_records.json').read_text());shadow=json.loads((D/'B0_shadow_until_fork.json').read_text());req=np.load(D/'requests.npz')['requests'];ident=json.loads((D/'identity.json').read_text())
assert ident['policy_MRO']==ident['baseline_policy_MRO'] and ident['tracker_MRO']==ident['baseline_tracker_MRO']
assert len(rows)==len(req)
for r in rows:assert r.get('route_side_hypothesis_is_binding') is False
forks=[r for r in shadow if r['first_request_fork']];assert len(forks)<=1
for i,r in enumerate(shadow):
 assert r['frame']==i and r['source_time_s']<=r['time_s']
 np.testing.assert_array_equal(r['candidate_request'],req[i])
 if not r['first_request_fork']:np.testing.assert_array_equal(r['B0_request'],req[i])
if forks:assert shadow[-1]==forks[0]
a=dict(decision='PASS_REAL_REQUEST_INTERVENTION_IDENTITY_ONLY' if forks else 'REJECT_NONINTERVENTION_ACTUAL_CANDIDATE_NOT_EXERCISED',actual_requests=len(req),actual_online_shadow_rows=len(shadow),first_request_fork=forks[0]['frame'] if forks else None,first_fork_record=forks[0] if forks else None,same_policy_and_tracker_source_MRO=True,all_actual_grid_diagnostics_nonbinding=True,no_request_state_reset=True,not_alternate_future_physics=True,not_full_navigation_causal_AB=True,no_NAV_admission=True,whole_research_complete=False)
(H/'original_E1_repeated_exposure_memory_intervention_assessment_v1.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps({k:v for k,v in a.items() if k!='first_fork_record'},indent=2))
