from pathlib import Path
import json,hashlib
H=Path(__file__).resolve().parent;source=H/'run_selfmask_rgb_navigation.py';s=source.read_text();changes=[]
def rep(a,b):
 global s
 assert s.count(a)==1,(a,s.count(a));s=s.replace(a,b);changes.append(dict(old=a,new=b))
rep('from masked_source_pair import paired_sources,ReadOnlyMaskShadow','from masked_source_pair import paired_sources\nfrom masked_memory_shadow import ReadOnlyCurrentMemoryShadow')
rep("H/'selfmask_rgb_navigation_protocol.json'","H/'masked_memory_navigation_protocol.json'")
rep("H/'selfmask_rgb_current_run'","H/'masked_memory_history_run'")
rep("episode_id='selfmask_rgb_e1_local2m_'+arm","episode_id='masked_memory_e1_local2m_'+arm")
rep('shadow=ReadOnlyMaskShadow(protocol)','shadow=ReadOnlyCurrentMemoryShadow(protocol)')
rep("shadow_started=time.perf_counter();old_current=dict(frame=current_packet['frame'],time_s=current_packet['time_s'],points_world=old_source_packet['world'],root_xyz=current_packet['root_xyz'])","shadow_started=time.perf_counter()")
rep('shadow.consume(old_source_packet,old_current,observation,goal)','shadow.consume(source_packet,current_packet,observation,goal)')
rep('same_raw_source_oldmask_shadow=True','same_masked_source_current_memory_shadow=True')
rep("'SELFMASK_RGB_LOCAL_COMPLETE'","'MASKED_MEMORY_LOCAL_COMPLETE'")
out=H/'run_masked_memory_navigation.py';out.write_text(s);reverse=s
for c in reversed(changes):assert reverse.count(c['new'])==1;reverse=reverse.replace(c['new'],c['old'])
assert reverse==source.read_text();(H/'masked_memory_runner_derivation.json').write_text(json.dumps(dict(base=str(source),base_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),candidate=str(out),candidate_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),reverse_exact=True,changes=changes,primary_memory_mode_given_by_protocol_arms='history'),indent=2)+'\n')
s=(H/'run_selfmask_rgb_navigation_stage.py').read_text().replace('selfmask_rgb_navigation','masked_memory_navigation').replace('selfmask_rgb_current_run','masked_memory_history_run');(H/'run_masked_memory_navigation_stage.py').write_text(s)
