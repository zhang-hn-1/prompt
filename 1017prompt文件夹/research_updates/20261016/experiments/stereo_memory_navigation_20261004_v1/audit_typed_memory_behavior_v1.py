"""One bounded post-close consumer/equivalence audit, no control changes."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
H=Path(__file__).resolve().parent;A=json.loads((H/'typed_hypothesis_memory_assessment_v2.json').read_text());rows={};physics={};summary={}
for arm in ['B0','C1']:
 D=H/f'typed_hypothesis_memory_{arm}_run_v1';r=json.loads((D/'policy_records.json').read_text());rows[arm]=r;physics[arm]=np.load(D/'physics.npz');birth=float(np.load(D/'immutable_possible_set_birth.npz')['time_s']);loss=next((x for x in r if x['local']['source_birth_time_s'] is not None and x['local']['past_model_hypothesis']==0 and x['local']['current_model_hypothesis']==0),None);changes=[]
 for i,x in enumerate(r):
  l=x['local']
  if i==0 or l['local_reference_version']!=r[i-1]['local']['local_reference_version']:changes.append(dict(frame=i,time_s=l['time_s'],version=l['local_reference_version'],model_cells=l['current_model_hypothesis']+l['past_model_hypothesis'],plan=l['plan'],reference_age_s=l['reference_age_s']))
 summary[arm]=dict(model_birth_time_s=birth,first_expired_model_frame=None if loss is None else loss['frame'],first_expired_model_time_s=None if loss is None else loss['local']['time_s'],reference_revision_events=changes,goal_result=A['arms'][arm],provenance_hypothesis_only=True)
n=min(len(rows['B0']),len(rows['C1']));requests={a:np.array([x['local']['request'] for x in rows[a][:n]]) for a in rows};diff=np.max(np.abs(requests['B0']-requests['C1']),axis=1);ticks=min(len(physics['B0']['applied_action']),len(physics['C1']['applied_action']));applied=np.max(np.abs(physics['B0']['applied_action'][:ticks]-physics['C1']['applied_action'][:ticks]),axis=1);nd=np.flatnonzero(diff>1e-6);ad=np.flatnonzero(applied>1e-6);meaningful_change=bool(len(nd) and len(ad));hyp_label_diff=[]
for i in range(n):
 b=rows['B0'][i]['local'];c=rows['C1'][i]['local']
 if b['past_model_hypothesis']!=c['past_model_hypothesis']:hyp_label_diff.append(i)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=summary,overlap_request_frames=n,request_frames_different_gt1e6=len(nd),first_request_difference_frame=None if not len(nd) else int(nd[0]),maximum_request_difference=float(diff.max()),applied_ticks_different_gt1e6=len(ad),first_applied_difference_tick=None if not len(ad) else int(ad[0]),model_label_difference_frames=len(hyp_label_diff),effective_executed_behavior_changed=meaningful_change,behavior_equivalent_despite_model_label_change=bool(hyp_label_diff and not meaningful_change),candidate_local_result=A['decision'],strict_causal_prefix_pass=A['strict_causal_AB_claim'],no_posthoc_gate_or_controller_change=True,not_formal35_or_whole_research_complete=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(H/'typed_memory_behavior_audit_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
