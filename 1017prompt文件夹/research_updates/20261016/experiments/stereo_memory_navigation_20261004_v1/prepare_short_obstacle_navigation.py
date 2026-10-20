from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
base=H/'run_masked_memory_navigation.py';s=base.read_text();changes=[]
def rep(a,b):
 global s
 assert s.count(a)==1,(a,s.count(a));s=s.replace(a,b);changes.append(dict(old=a,new=b))
rep('from masked_memory_shadow import ReadOnlyCurrentMemoryShadow','from masked_memory_shadow import ReadOnlyCurrentMemoryShadow\nfrom same_stream_memory_shadow import ReadOnlyMemoryShadow\nfrom short_obstacle_task import short_obstacle_endpoints')
rep("H/'masked_memory_navigation_protocol.json'","H/'short_obstacle_navigation_protocol.json'")
rep("H/'masked_memory_history_run'","H/('short_obstacle_'+arm+'_run')")
old="episode=dataclasses.replace(episode,episode_id='masked_memory_e1_local2m_'+arm,goal_position_world_m=tuple(short_goal(episode.start_position_world_m,episode.goal_position_world_m)),euclidean_distance_m=min(2.,episode.euclidean_distance_m),oracle_path_length_m=max(2.,min(episode.oracle_path_length_m,2.)))"
rep(old,"newstart,newgoal=short_obstacle_endpoints(episode.start_position_world_m,episode.goal_position_world_m)\n    episode=dataclasses.replace(episode,episode_id='short_obstacle_e1_'+arm,start_position_world_m=tuple(newstart),goal_position_world_m=tuple(newgoal),euclidean_distance_m=float(np.linalg.norm(newgoal[:2]-newstart[:2])))")
rep("task='explicit_flat_ground_prior_unknown_exploration_local2m'","task='explicit_ground_prior_short_obstacle_diagnostic'")
rep('not_original_complete_E1=True','not_original_complete_E1=True,oracle_path_metadata_inherited_not_recomputed=True,SPL_not_used_for_this_diagnostic=True')
rep('shadow=ReadOnlyCurrentMemoryShadow(protocol)',"shadow=ReadOnlyMemoryShadow(protocol) if arm=='current' else ReadOnlyCurrentMemoryShadow(protocol)")
rep("shadow.consume(source_packet,current_packet,observation,goal);shadow.records[-1]['shadow_wall_s']=time.perf_counter()-shadow_started", "shadow.consume(source_packet,current_packet,observation,goal);shadow.records[-1].update(shadow_wall_s=time.perf_counter()-shadow_started,source_filter='dynamic_current_joint_CAD',memory_role='history' if arm=='current' else 'current')")
rep('same_masked_source_current_memory_shadow=True',"same_masked_source_opposite_memory_shadow=True,primary_memory=arm,shadow_memory='history' if arm=='current' else 'current'")
rep("'MASKED_MEMORY_LOCAL_COMPLETE'","'SHORT_OBSTACLE_DIAGNOSTIC_COMPLETE'")
out=H/'run_short_obstacle_navigation.py';assert not out.exists();out.write_text(s);rev=s
for c in reversed(changes):assert rev.count(c['new'])==1;rev=rev.replace(c['new'],c['old'])
assert rev==base.read_text()
(H/'short_obstacle_runner_derivation.json').write_text(json.dumps(dict(base=str(base),base_sha256=sha(base),candidate=str(out),candidate_sha256=sha(out),reverse_exact=True,changes=changes,scientific_scope='new declaredshort task same frozenbaseline; two memoryarms/opposite readonlyshadow'),indent=2)+'\n')
stage=(H/'run_masked_memory_navigation_stage.py').read_text().replace('masked_memory_navigation','short_obstacle_navigation').replace("H/'masked_memory_history_run.log'","H/('short_obstacle_'+arm+'_run.log')").replace("H/'masked_memory_history_run'","H/('short_obstacle_'+arm+'_run')").replace('maximum_sequences=1','maximum_sequences=2').replace("entry=dict(arm=arm,status='running',start_utc=now())","entry=dict(arm=arm,primary_memory=arm,shadow_memory='history' if arm=='current' else 'current',status='running',start_utc=now())")
(H/'run_short_obstacle_navigation_stage.py').write_text(stage)
p=json.loads((H/'masked_memory_navigation_protocol.json').read_text());p.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=['current','history'],maximum_sequences=2,local_goal_m=2.6,diagnostic_task='symmetric2mshortening_originalE1_obstacle',endpoint_definition='start+=2*XYunit;goal-=2*XYunit; originalheight/yaw',accepted_controller='frozenCADPriorRGBPolicy originalunknowncap.3',terminal_candidate_used=False,old_original_fullgoal_failures_preserved=True,oracle_path_metadata_inherited_not_recomputed=True,SPL_not_used=True,shadow_memory_by_arm={'current':'history_readonly','history':'current_readonly'},offline_declared_endpoint_legality={'nearest_occupied_centre_start_m':1.,'nearest_occupied_centre_goal_m':1.,'safe_start_prior_m':.7,'halfcell_diag_m':.1*2**.5/2,'offline_notroute_guidance':True})
sources=set(p['source_sha256']);sources.update(str(f) for f in [out,H/'short_obstacle_task.py',H/'test_short_obstacle_task.py',H/'prepare_short_obstacle_navigation.py',H/'run_short_obstacle_navigation_stage.py',H/'short_obstacle_runner_derivation.json',W/'docs/superpowers/specs/2026-10-07-short-obstacle-navigation-design.md',W/'docs/superpowers/plans/2026-10-07-short-obstacle-navigation.md']);p['source_sha256']={f:sha(f) for f in sorted(sources)}
(H/'short_obstacle_navigation_protocol.json').write_text(json.dumps(p,indent=2)+'\n');compile(s,str(out),'exec');print('presealed',len(sources),p['arms'])
