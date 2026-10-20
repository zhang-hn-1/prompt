"""Historical measured-boundary preflight only, no policy act/future feedback."""
from pathlib import Path
import json,hashlib,datetime
from raster_consistent_bridge import bridge_screen
H=Path(__file__).resolve().parent;D=H/'typed_hypothesis_memory_C1_run_v1';r=json.loads((D/'policy_records.json').read_text());rows=[]
for x in r:
 l=x['local'];decision=l['reference_decision']
 if not l['guard_measured_this_call']:continue
 t=float(l['time_s']);m=dict(time_s=decision['measured_time_s'],guard=decision['guard']);s=bridge_screen(m,t,.1)
 rows.append(dict(frame=x['frame'],time_s=t,**s,old_plan_status=l['plan'].get('status'),old_reference_decision=decision['decision']))
changes=[x for x in rows if x['candidate_retain']!=x['original_retain']];first=next(x for x in rows if x['frame']==200);passed=first['candidate_retain'] and first['alias_only_current_bridge'] and all(x['candidate_retain'] for x in rows if x['original_retain'])
a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_CURRENT_BRIDGE_PREFLIGHT_ONLY' if passed else 'REJECT_BRIDGE_PREFLIGHT',boundary_calls=len(rows),changed_historical_boundaries=changes,frame200=first,all_old_accepted_boundaries_preserved=all(x['candidate_retain'] for x in rows if x['original_retain']),old_data_used_for_design_not_fresh=True,no_stateful_act_recall=True,no_future_feedback_or_GT_read=True,old_reject_gate_not_rescored=True,no_new_actual_navigation_or_formal35=True,whole_research_complete=False,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),helper_sha256=hashlib.sha256((H/'raster_consistent_bridge.py').read_bytes()).hexdigest())
(H/'raster_bridge_applicability_v1.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
