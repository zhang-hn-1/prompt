from pathlib import Path
import json,hashlib
H=Path(__file__).resolve().parent;base=H/'run_masked_memory_navigation.py';s=base.read_text();changes=[]
def rep(a,b):
 global s
 assert s.count(a)==1,(a,s.count(a));s=s.replace(a,b);changes.append(dict(old=a,new=b))
rep('from masked_memory_shadow import ReadOnlyCurrentMemoryShadow','from nominal_unknown_cap import with_nominal_unknown_cap,ReadOnlyHistorySlowShadow')
rep("H/'masked_memory_navigation_protocol.json'","H/'nominal_cap_navigation_protocol.json'")
rep("H/'masked_memory_history_run'","H/'nominal_cap_history_run'")
rep("episode_id='masked_memory_e1_local2m_'+arm","episode_id='nominal_cap_e1_local2m_'+arm")
rep('policy=PriorRGBPolicy(protocol,memory_mode=arm)','policy=with_nominal_unknown_cap(PriorRGBPolicy)(protocol,memory_mode=arm)')
rep('shadow=ReadOnlyCurrentMemoryShadow(protocol)','shadow=ReadOnlyHistorySlowShadow(protocol)')
rep("shadow.consume(source_packet,current_packet,observation,goal);shadow.records[-1]['shadow_wall_s']=time.perf_counter()-shadow_started", """shadow.consume(source_packet,current_packet,observation,goal);shadow.records[-1]['shadow_wall_s']=time.perf_counter()-shadow_started
        assert policy.records[-1]['uncapped_request']==shadow.records[-1]['local']['uncapped_request'] and boundary_state(policy.tracker)==boundary_state(shadow.policy.tracker)
        assert np.array_equal(policy.grid.labels,shadow.policy.grid.labels) and np.array_equal(policy.grid.memory.floor_time,shadow.policy.grid.memory.floor_time) and np.array_equal(policy.grid.memory.hazard_time,shadow.policy.grid.memory.hazard_time)
        np.testing.assert_equal(policy.tracker._reference_xy,shadow.policy.tracker._reference_xy)
        shadow.records[-1]['same_parent_source_state_verified']=True""")
rep('same_masked_source_current_memory_shadow=True','same_masked_source_history_slowcap_shadow=True')
rep('shared_unknown_request_main_cap_rad_s=.3','shared_unknown_request_main_cap_rad_s=1.')
rep("'MASKED_MEMORY_LOCAL_COMPLETE'","'NOMINAL_CAP_LOCAL_COMPLETE'")
out=H/'run_nominal_cap_navigation.py';out.write_text(s);reverse=s
for c in reversed(changes):assert reverse.count(c['new'])==1;reverse=reverse.replace(c['new'],c['old'])
assert reverse==base.read_text();(H/'nominal_cap_runner_derivation.json').write_text(json.dumps(dict(base=str(base),base_sha256=hashlib.sha256(base.read_bytes()).hexdigest(),candidate=str(out),candidate_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),reverse_exact=True,changes=changes,scientific_change='unknown main cap only .3→1.0'),indent=2)+'\n')
s=(H/'run_masked_memory_navigation_stage.py').read_text().replace('masked_memory_navigation','nominal_cap_navigation').replace('masked_memory_history_run','nominal_cap_history_run');(H/'run_nominal_cap_navigation_stage.py').write_text(s)
