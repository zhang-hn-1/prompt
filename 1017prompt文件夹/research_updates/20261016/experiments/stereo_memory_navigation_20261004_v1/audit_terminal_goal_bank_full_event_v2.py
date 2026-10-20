from pathlib import Path
import numpy as np,json,hashlib,datetime
from terminal_bank_effect_common_prefix import audit
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'terminal_goal_bank_full_event_v2';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
trace=json.loads((H/'native_terminal_factual_chain_v2/trace.json').read_text());es=json.loads((D/'execution_states.json').read_text());ph=dict(np.load(D/'physics.npz'));r=audit(trace,es,ph)
old=json.loads((H/'terminal_goal_bank_full_event_v1/assessment.json').read_text());assert r['ticks']==old['ticks'] and r['first_applied_divergence_index']==old['first_applied_divergence_index']==2783
last=r['last_request_frame'];pidx=r['first_applied_divergence_index'];truncated=[row for row in trace if row['frame']<=last];np.testing.assert_equal(audit(truncated,es[:last+1],{k:v[:pidx+1] for k,v in ph.items()}),r)
poisoned=json.loads(json.dumps(trace))
for row in poisoned:
 if row['frame']>last:row['request']=[float('nan')]*10;row['request_diagnostics']['request_source']='poison'
pp={k:v.copy() for k,v in ph.items()}
for k,v in pp.items():
 if len(v)>pidx+1:v[pidx+1:]=999
np.testing.assert_equal(audit(poisoned,es,pp),r)
r.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_EXISTING_CANDIDATE_EXECUTION_EFFECT_ONLY',whole_research_complete=False,not_physical_prediction_or_navigation=True,future_replacement_exact=True,future_truncation_exact=True,old_five_tick_scope_preserved=True,input_sha256={str(f):sha(f) for f in [H/'native_terminal_factual_chain_v2/trace.json',D/'execution_states.json',D/'physics.npz',H/'terminal_goal_bank_source.py',H/'terminal_goal_bank_stateful_audit_v2/assessment.json',H.parent.parent/'docs/superpowers/plans/2026-10-08-terminal-bank-full-event-effect.md']},source_sha256={str(f):sha(f) for f in [Path(__file__),H/'terminal_bank_effect_common_prefix.py']})
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='ticks'},indent=2))
