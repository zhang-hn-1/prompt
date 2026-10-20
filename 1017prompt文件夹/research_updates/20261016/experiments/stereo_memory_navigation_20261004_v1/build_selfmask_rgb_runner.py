from pathlib import Path
import hashlib,json
H=Path(__file__).resolve().parent;p=H/'run_prior_rgb_navigation.py';s=p.read_text();changes=[]
def replace(old,new):
 global s
 assert s.count(old)==1,(old[:80],s.count(old));s=s.replace(old,new);changes.append(dict(old=old,new=new))
replace('from prior_rgb_navigation import PriorRGBPolicy,short_goal,require_boolean_confidence','from prior_rgb_navigation import PriorRGBPolicy,short_goal,require_boolean_confidence\nfrom masked_source_pair import paired_sources,ReadOnlyMaskShadow\nfrom dynamic_self_mask import RobotSelfMask\nfrom guarded_reference import boundary_state\nimport copy')
replace("H/'prior_rgb_navigation_protocol.json'","H/'selfmask_rgb_navigation_protocol.json'")
replace("out=H/('prior_rgb_'+arm+'_run')","out=H/'selfmask_rgb_current_run'")
replace("'policy_inputs']","'policy_inputs','masks','old_sensor_packets']")
replace("episode_id='prior_rgb_e1_local2m_'+arm","episode_id='selfmask_rgb_e1_local2m_'+arm")
replace('current_packet=None;source_packet=None;depth_model=None','current_packet=None;source_packet=None;depth_model=None;old_source_packet=None;self_masker=None')
replace('nonlocal current_packet,source_packet,depth_model','nonlocal current_packet,source_packet,depth_model,old_source_packet,self_masker')
old="source_packet=point_packet(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(**poses,root_position_world_m=np.asarray(p),time_s=t),maskdata['left'],maskdata['right'])"
new="""if self_masker is None:self_masker=RobotSelfMask()
                old_source_packet,source_packet,current_masks=paired_sources(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(**poses,root_position_world_m=np.asarray(p),time_s=t),step.observation.proprio_history,self_masker,t)
                np.savez_compressed(out/'masks'/f'frame_{i:03d}.npz',**current_masks,current_joints=step.observation.proprio_history[-1,9:12],frame=i,time_s=t)
                np.savez_compressed(out/'old_sensor_packets'/f'source_{i:03d}.npz',**old_source_packet)"""
replace(old,new)
replace('    act=policy.act','''    shadow=ReadOnlyMaskShadow(protocol);reset=policy.reset
    def recorded_reset(context):
        reset(context);shadow.reset(context)
    policy.reset=recorded_reset
    act=policy.act''')
replace("        np.savez_compressed(out/'policy_inputs'", """        before_shadow=copy.deepcopy(boundary_state(policy.tracker));action_copy=action.copy();clock_before=runtime._clock_metadata()
        physical_before=[np.asarray(v).copy() for v in (*runtime._robot.get_world_pose(),runtime._robot.get_linear_velocity(),runtime._robot.get_angular_velocity(),runtime._robot.get_joint_positions(),runtime._robot.get_joint_velocities())]
        shadow_started=time.perf_counter();old_current=dict(frame=current_packet['frame'],time_s=current_packet['time_s'],points_world=old_source_packet['world'],root_xyz=current_packet['root_xyz'])
        shadow.consume(old_source_packet,old_current,observation,goal);shadow.records[-1]['shadow_wall_s']=time.perf_counter()-shadow_started
        physical_after=[np.asarray(v).copy() for v in (*runtime._robot.get_world_pose(),runtime._robot.get_linear_velocity(),runtime._robot.get_angular_velocity(),runtime._robot.get_joint_positions(),runtime._robot.get_joint_velocities())]
        assert boundary_state(policy.tracker)==before_shadow and np.array_equal(action,action_copy) and runtime._clock_metadata()==clock_before and all(np.array_equal(a,b) for a,b in zip(physical_before,physical_after))
        np.savez_compressed(out/'policy_inputs'""")
replace("('policy_records',policies)]","('policy_records',policies),('shadow_records',shadow.records)]")
replace("local_goal_m=2.,memory_arm=arm","local_goal_m=2.,memory_arm=arm,dynamic_selfmask=True,same_raw_source_oldmask_shadow=True,shadow_executed=False")
replace("'PRIOR_RGB_LOCAL_COMPLETE'","'SELFMASK_RGB_LOCAL_COMPLETE'")
q=H/'run_selfmask_rgb_navigation.py';q.write_text(s);reverse=s
for c in reversed(changes):assert reverse.count(c['new'])==1;reverse=reverse.replace(c['new'],c['old'])
assert reverse==p.read_text();(H/'selfmask_rgb_runner_derivation.json').write_text(json.dumps(dict(base=str(p),base_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),candidate=str(q),candidate_sha256=hashlib.sha256(q.read_bytes()).hexdigest(),reverse_exact=True,changes=changes),indent=2)+'\n')
s=(H/'run_prior_rgb_navigation_stage.py').read_text().replace('prior_rgb_navigation','selfmask_rgb_navigation').replace("H/('prior_rgb_'+arm+'_run.log')","H/'selfmask_rgb_current_run.log'").replace("H/('prior_rgb_'+arm+'_run')","H/'selfmask_rgb_current_run'").replace('maximum_sequences=2','maximum_sequences=1')
(H/'run_selfmask_rgb_navigation_stage.py').write_text(s)
