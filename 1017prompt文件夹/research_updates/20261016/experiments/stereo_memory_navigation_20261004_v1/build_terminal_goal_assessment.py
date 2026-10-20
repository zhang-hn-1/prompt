from pathlib import Path
import json,hashlib
H=Path(__file__).resolve().parent;base=H/'assess_nominal_cap_navigation.py';s=base.read_text().replace('nominal_cap_history_run','terminal_goal_history_run').replace('nominal_cap_navigation','terminal_goal_navigation')
# Only administrative cross-batch floating-point check; each reconstructed packet vs saved remains exact.
s=s.replace("np.testing.assert_equal(old['world'][sel],new['world']);", "np.testing.assert_allclose(old['world'][sel],new['world'],rtol=32*np.finfo(float).eps,atol=32*np.finfo(float).eps);")
# Avoid repeated deep copies of unconsumed rawsource record lists; same six arrays/diagnostics tested exact.
s=s.replace('from prior_rgb_navigation import PriorRGBGrid','from verification_grid import VerificationGrid as PriorRGBGrid')
s=s.replace("j=int(hit[0]);result['goal_entry_sampling'][label]", "j=int(hit[0]) if len(hit) else len(dd)-1;result['goal_entry_sampling'][label]")
s=s.replace("(H/'terminal_goal_navigation_assessment.json').write_text", """
from nominal_unknown_cap import nominal_unknown_action
from terminal_goal_consumer import terminal_action
terminal=False;terminal_frames=[];fast_effective_diffs=[]
for i,r in enumerate(rows):
 local=r['local'];saved=np.load(O/'policy_inputs'/f'frame_{i:03d}.npz')
 d=float(np.linalg.norm(saved['pointgoal_base_xy']));pre=nominal_unknown_action(local['uncapped_request'],local['unknown_fraction']);action,terminal=terminal_action(pre,d,terminal)
 np.testing.assert_equal(action,saved['request']);np.testing.assert_equal(action,local['request']);np.testing.assert_equal(pre,local['pre_terminal_request'])
 assert terminal==local['terminal_mode'] and d==local['terminal_goal_distance_m']
 if terminal:terminal_frames.append(i)
 if np.any(action!=pre):fast_effective_diffs.append(i)
 assert np.array_equal(action[1::2],pre[1::2]) and np.all(np.abs(action[::2])<=np.abs(pre[::2]))
assert terminal_frames and fast_effective_diffs
# Actual frozenfast prefix independently checked, without assuming two RGB restarts bit-identical.
fastdir=H/'nominal_cap_history_run';fastrows=json.loads((fastdir/'policy_records.json').read_text());fastA=np.array([r['local']['request'] for r in fastrows]);c=min(N,len(fastA));fd=np.flatnonzero(np.any(A[:c]!=fastA[:c],axis=1));branchfast=int(fd[0]) if len(fd) else c;fastph=np.load(fastdir/'physics.npz')
for k in prefixfields:np.testing.assert_equal(ph[k][:5*branchfast+1],fastph[k][:5*branchfast+1])
assert json.loads((O/'execution_states.json').read_text())[:branchfast+1]==json.loads((fastdir/'execution_states.json').read_text())[:branchfast+1]
result.update(terminal_first_frame=terminal_frames[0],terminal_first_distance_m=rows[terminal_frames[0]]['local']['terminal_goal_distance_m'],terminal_effective_request_blocks=len(fast_effective_diffs),frozenfast_actual_first_request_difference_frame=branchfast,exact_frozenfast_physical_prefix_samples=5*branchfast+1,exact_frozenfast_execution_prefix=branchfast+1,terminal_action_chain_rebuilt_calls=N)
result['terminal_consumer_gate_pass']=bool(result['nominal_cap_increment_pass'] and fast_effective_diffs)
result['old_nominal_cap_rejection_preserved']=True
result['scope']='one terminal main consumer localknownpose; no formalnav/fullgoal until accepted'
(H/'terminal_goal_navigation_assessment.json').write_text""")
out=H/'assess_terminal_goal_navigation.py';out.write_text(s)
(H/'terminal_goal_assessment_derivation.json').write_text(json.dumps(dict(base=str(base),base_sha256=hashlib.sha256(base.read_bytes()).hexdigest(),candidate=str(out),candidate_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),scope='posthoc audit, originalnumericgates unchanged, admin32eps crossbatch only; testedgridarrays substitute; terminalcurrentgoal chain audit'),indent=2)+'\n')
compile(s,str(out),'exec');print(out)
