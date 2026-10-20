from pathlib import Path
import hashlib,json
H=Path(__file__).resolve().parent;base=H/'assess_terminal_goal_navigation.py';s=base.read_text().split("base=H/'masked_memory_history_run';")[0].replace('terminal_goal_history_run','terminal_goal_full_history_run').replace('terminal_goal_navigation_protocol','terminal_goal_full_navigation_protocol').replace('terminal_goal_navigation_status','terminal_goal_full_navigation_status')
s+='''
from nominal_unknown_cap import nominal_unknown_action
from terminal_goal_consumer import terminal_action
terminal=False;terminal_frames=[];effective=[]
for i,r in enumerate(rows):
 local=r['local'];saved=np.load(O/'policy_inputs'/f'frame_{i:03d}.npz');d=float(np.linalg.norm(saved['pointgoal_base_xy']));pre=nominal_unknown_action(local['uncapped_request'],local['unknown_fraction']);action,terminal=terminal_action(pre,d,terminal)
 np.testing.assert_equal(action,saved['request']);np.testing.assert_equal(action,local['request']);np.testing.assert_equal(pre,local['pre_terminal_request']);assert terminal==local['terminal_mode'] and d==local['terminal_goal_distance_m']
 np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],action.reshape(5,2))
 if terminal:terminal_frames.append(i)
 if np.any(action!=pre):effective.append(i)
 assert np.array_equal(action[1::2],pre[1::2])
assert all(s['same_parent_source_state_verified'] for s in shadow)
nonzero=np.flatnonzero(np.any(A[:,::2]!=0,axis=1));move=int(nonzero[0]) if len(nonzero) else N;movement_cross=cross[move:]
result.update(original_full_goal_pass=result['local_task_pass'],goal_displacement_m=float(np.linalg.norm(goal-start)),terminal_action_chain_rebuilt_calls=N,terminal_first_frame=terminal_frames[0] if terminal_frames else None,terminal_effective_request_blocks=len(effective),movement_cross_RMS_m=float(np.sqrt(np.mean(movement_cross**2))),movement_cross_peak_m=float(np.abs(movement_cross).max()),max_goal_direction_progress_m=float(progress.max()),acceptedlocal_sha256=p['local_acceptance_sha256'],controlled_single_terminal_effect_full_AB_not_measured=True,old_full_bases_remainfailed=True,scope='ONE sameacceptedconsumer originalfullgoal35s diagnostic; notformalnav or pureterminal causal fullAB')
result.pop('local_task_pass');result['full_goal_triggered']=True
(H/'terminal_goal_full_navigation_assessment.json').write_text(json.dumps(result,indent=2)+'\\n');print(json.dumps({k:v for k,v in result.items() if k!='paired_source_frames'},indent=2))
'''
out=H/'assess_terminal_goal_full_navigation.py';out.write_text(s);compile(s,str(out),'exec');(H/'terminal_goal_full_assessment_derivation.json').write_text(json.dumps(dict(base=str(base),base_sha256=hashlib.sha256(base.read_bytes()).hexdigest(),candidate=str(out),candidate_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),scope='same source/memory/clock audit, variableepisode denominators; fullgoal35s gate replaces localstageperformance only for NEW full diagnostic; old localgates unchanged'),indent=2)+'\n');print(out)
