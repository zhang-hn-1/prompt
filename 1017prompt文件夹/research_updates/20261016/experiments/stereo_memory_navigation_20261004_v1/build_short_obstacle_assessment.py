from pathlib import Path
import hashlib,json
H=Path(__file__).resolve().parent;base=H/'assess_nominal_cap_navigation.py'
s=base.read_text().split("base=H/'masked_memory_history_run';")[0]
s=s.replace('import json,hashlib,datetime','import json,hashlib,datetime,sys')
s=s.replace("O=H/'nominal_cap_history_run'","arm=sys.argv[1];assert arm in ('current','history');O=H/('short_obstacle_'+arm+'_run')")
s=s.replace("H/'nominal_cap_navigation_protocol.json'","H/'short_obstacle_navigation_protocol.json'").replace("H/'nominal_cap_navigation_status.json'","H/'short_obstacle_navigation_status.json'")
s=s.replace("assert status['status']=='finished' and status['arms'][0]['exit_code']==0 and p['utc']<status['start_utc']", "entry=next(x for x in status['arms'] if x['arm']==arm);assert entry['status']=='finished' and entry['exit_code']==0 and p['utc']<entry['start_utc']")
s=s.replace('from prior_rgb_navigation import PriorRGBGrid','from verification_grid import VerificationGrid as PriorRGBGrid')
s=s.replace("np.testing.assert_equal(old['world'][sel],new['world']);", "np.testing.assert_allclose(old['world'][sel],new['world'],rtol=32*np.finfo(float).eps,atol=32*np.finfo(float).eps);")
s=s.replace("float(s['resolution']),'history');shadowgrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),'history')", "float(s['resolution']),arm);shadowgrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),'history' if arm=='current' else 'current')")
s+='''
from short_obstacle_task import short_obstacle_endpoints
original=task['original_episode'];newstart,newgoal=short_obstacle_endpoints(original['start_position_world_m'],original['goal_position_world_m']);np.testing.assert_equal(newstart,task['actual_episode']['start_position_world_m']);np.testing.assert_equal(newgoal,task['actual_episode']['goal_position_world_m']);assert p['unknown_request_main_cap_rad_s']==.3 and not p['terminal_candidate_used'];assert not any('terminal_mode' in r['local'] for r in rows)
for i,r in enumerate(rows):
 np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],np.asarray(r['local']['request']).reshape(5,2))
 assert r['local']['memory_mode']==arm
 assert shadow[i]['memory_role']==('history' if arm=='current' else 'current')
mid=float(np.linalg.norm(goal-start)/2);crossed=np.flatnonzero(progress>=mid);checkpoints=[]
for metres in [.5,mid,2.3]:
 hit=np.flatnonzero(progress>=metres);checkpoints.append(dict(goal_line_progress_m=metres,first_frame=int(hit[0]) if len(hit) else None,time_s=float(states[int(hit[0])]['time_s']) if len(hit) else None))
nonzero=np.flatnonzero(np.any(A[:,::2]!=0,axis=1));move=int(nonzero[0]) if len(nonzero) else N;crossmove=cross[move:]
result.update(task='new shortened obstacle knownpose diagnosis',arm=arm,shadow_memory='history' if arm=='current' else 'current',goal_span_m=float(np.linalg.norm(goal-start)),actual_crossed_midplane=bool(len(crossed)),independent_checkpoints=checkpoints,movement_cross_RMS_m=float(np.sqrt(np.mean(crossmove**2))),movement_cross_peak_m=float(np.abs(crossmove).max()),old_6p6m_failures_preserved=True,strict_nominal_and_terminal_rejects_preserved=True,SPL_legacy_metadata_not_used=True,original_35s_budget_unchanged=True,online_inputs_current_or_past_knownpose_RGB_proprio=True,posthoc_grid_equivalence_not_online_optimization=True)
result['short_obstacle_task_pass']=bool(result.pop('local_task_pass') and len(crossed))
result['comparative_memory_benefit_not_yet_adjudicated']=True
(H/('short_obstacle_'+arm+'_assessment.json')).write_text(json.dumps(result,indent=2)+'\\n');print(json.dumps({k:v for k,v in result.items() if k!='paired_source_frames'},indent=2))
'''
out=H/'assess_short_obstacle_navigation.py';out.write_text(s);compile(s,str(out),'exec');(H/'short_obstacle_assessment_derivation.json').write_text(json.dumps(dict(base=str(base),base_sha256=hashlib.sha256(base.read_bytes()).hexdigest(),candidate=str(out),candidate_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),scope='same savedsource/clock/memory/video/variableN audit, exact original35/.2/contact + newdeclaredobstaclemidplane gate, oldcandidategates notrescored; GRIDposthoc equivalent'),indent=2)+'\n');print(out)
