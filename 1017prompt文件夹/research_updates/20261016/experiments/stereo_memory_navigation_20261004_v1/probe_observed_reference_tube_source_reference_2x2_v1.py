"""Four current-input counterfactuals, no alternate future physics or actor update."""
from pathlib import Path
import json,copy,numpy as np,datetime
H=Path(__file__).resolve().parent
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),question='At first crossrun action difference211, distinguish observed-map/source-history vs existing-controller/reference state under identical actual measured input',maximum_counterfactual_calls=4,currentframe=211,only_sealed_runs=True,no_future_data_consumed=True,not_new_physics=True,no_threshold_changes=True)
(H/'observed_reference_tube_source_reference_2x2_protocol_v1.json').write_text(json.dumps(protocol,indent=2)+'\n')
s=(H/'verify_front_view_handoff_policy_causal_v2.py').read_text();s=s[:s.index('full=run(')]
needle=";action=acq.choose(t,state['position'][:2],float(np.linalg.norm(observation.proprio_history[-1,:2])),src)"
assert needle in s;s=s.replace(needle,"\n  if i==stop:return policy,observation,goal,src,packet\n  action=acq.choose(t,state['position'][:2],float(np.linalg.norm(observation.proprio_history[-1,:2])),src)")
ns={'__file__':str(H/'verify_front_view_handoff_policy_causal_v2.py')};exec(compile(s,'<frozen B0 current-prefix>','exec'),ns)
prepared=[]
for name in ['front_view_handoff_navigation_run_v1','observed_reference_tube_navigation_run_v1']:
 D=H/name;ns.update(D=D,states=json.loads((D/'states.json').read_text()),sources=json.loads((D/'source_rows.json').read_text()),rows=json.loads((D/'policy_records.json').read_text()),actual=np.load(D/'requests.npz')['requests'],angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'])
 prepared.append(ns['run'](211,True))
pa,oa,ga,sa,ca=prepared[0];pb,ob,gb,sb,cb=prepared[1]
for k in ['proprio_history','action_history','position_world_xy','quaternion_world_wxyz_history']:np.testing.assert_array_equal(getattr(oa,k),getattr(ob,k))
np.testing.assert_array_equal(ga,gb)
results=[]
for ptag,pbase in [('A',pa),('B',pb)]:
 for gtag,gbase,src in [('A',pa.grid,sa),('B',pb.grid,sb)]:
  p=copy.deepcopy(pbase);p.grid=copy.deepcopy(gbase);p.set_source_packet(copy.deepcopy(src));p.set_current_packet(copy.deepcopy(ca));refbefore=np.asarray(p.tracker._reference_xy).copy();action=p.act(copy.deepcopy(oa),copy.deepcopy(ga));r=dict(controller_reference_state=ptag,map_source_history_state=gtag,request=action.tolist(),reference_before=refbefore.tolist(),reference_after=np.asarray(p.tracker._reference_xy).tolist(),reference_decision=p.records[-1].get('reference_decision'),nominal_reference_version=p.local_reference_version)
  if ptag==gtag:
   expected=np.load(H/('front_view_handoff_navigation_run_v1' if ptag=='A' else 'observed_reference_tube_navigation_run_v1')/'requests.npz')['requests'][211];np.testing.assert_array_equal(action,expected);r['factual_request_exact']=True
  results.append(r)
result=dict(protocol=protocol,measurements_same_exact=True,factual_AA_BB_requests_exact=True,counterfactuals=results,not_alternate_future_truth=True,not_actor_change=True,old_candidate_nonintervention_preserved=True,whole_research_complete=False)
(H/'observed_reference_tube_source_reference_2x2_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print([(r['controller_reference_state'],r['map_source_history_state'],r['request'][:2]) for r in results])
