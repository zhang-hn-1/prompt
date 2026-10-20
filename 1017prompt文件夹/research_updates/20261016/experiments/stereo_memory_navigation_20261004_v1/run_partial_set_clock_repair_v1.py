"""Same-version B0/C1, one source-readiness interface; privileged local diagnostic."""
from pathlib import Path
import json,hashlib,sys,traceback,time,datetime,dataclasses
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
import scripts.run_pointgoal_benchmark as runner
from benchmark.artifacts import write_rgb_video
from benchmark.runner import run_episode,RunnerConfig
from scripts.camera_rig import camera_rig_metadata
from execution_snapshot import export_execution
from complete_capture import complete_capture
from render_flush import fixed_render_flush
from prior_rgb_navigation import PriorRGBPolicy,short_goal,require_boolean_confidence
from masked_source_pair import paired_sources
from masked_memory_shadow import ReadOnlyCurrentMemoryShadow
from same_stream_memory_shadow import ReadOnlyMemoryShadow
from short_obstacle_task import short_obstacle_endpoints
from dynamic_self_mask import RobotSelfMask
from guarded_reference import boundary_state
import copy
from fast_stereo import FastStereo
from dense_source_points import point_packet
from qualified_hazard_consumer import QualifiedHazardConsumer,QualifiedHazardGrid
from past_point_rgb_memory import PastPointRGBMemory
from earlier_source_opportunity import EarlierSourceOpportunity
from static_seed_belief import StaticSeedBeliefGrid,SafeStartBootstrap
from partial_cue_source_v2 import PartialCueSetSource
import inspect
from stereo_geometry import restore_native
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import install_runtime
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from render_boundary import hide_diagnostic_markers,marker_visibility
sys.path.insert(0,str(H.parent/'clean_visual_hard0_intervention_20261003_v1'))
from clock_sync import ensure_current_frames
sys.path.insert(0,str(H.parent/'visible_surface_connectivity_20261004_v1'))
from rig_inputs import validate_capture,validate_mask_identity
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()

def main():
    config=json.loads((H/'partial_set_clock_repair_protocol_v1.json').read_text())
    for f,s in config['source_sha256'].items():assert sha(f)==s,f
    actor=json.loads((H/'partial_set_uncertainty_actor_v1.json').read_text());assert set(actor)=={'schema','radius_m','maximum_radius_m','method','fit_sha256','is_model_hypothesis_not_observation'};assert actor['is_model_hypothesis_not_observation'];partial_source=PartialCueSetSource(actor['radius_m'])
    variant=sys.argv[1];assert variant in config['variants'];arm='current';out=H/'partial_set_clock_repair_run_v1';out.mkdir(exist_ok=False)
    for name in ['online_inputs','known_poses','offline_truth','history','sensor_packets','source_predictions','maps','local_plans','policy_inputs','masks','old_sensor_packets','qualified_sensor_packets','appearance_old_packets','appearance_predictions','possible_sets','partial_class_masks']:(out/name).mkdir()
    episode=next(e for e in runner.load_manifest(config['manifest']).episodes if e.episode_id==config['episode_id'])
    occupancy=H/'controlled_obstacle_fixture_v1/occupancy.npz'
    original_episode=episode
    newstart=np.array([-2.2,0.,.45]);newgoal=np.array([2.2,0.,.45])
    episode=dataclasses.replace(episode,episode_id='controlled_static_obstacle_v1',scene_uri='${SCENE_ROOT}/single_cylinder.usda',scene_sha256=sha(H/'controlled_obstacle_fixture_v1/single_cylinder.usda'),occupancy_sha256=sha(occupancy),start_quaternion_world_wxyz=(1.,0.,0.,0.),start_position_world_m=tuple(newstart),goal_position_world_m=tuple(newgoal),euclidean_distance_m=4.4)
    (out/'task_episode.json').write_text(json.dumps(dict(task='controlled_initial_heading_source_only_8s',original_episode=original_episode.to_dict(),actual_episode=episode.to_dict(),not_original_complete_E1=True,oracle_path_metadata_inherited_not_recomputed=True,SPL_not_used_for_this_diagnostic=True),indent=2)+'\n')
    validate_mask_identity(json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text()))
    maskdata=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    yy,xx=np.mgrid[0:480:4,0:640:4]
    rays=np.column_stack(((xx.ravel()-319.5)/584.8888888888889,(yy.ravel()-239.5)/584.8888888888889,np.ones(xx.size)))
    clocks=[];states=[];execution_states=[];contacts=[];markers=[];policies=[];current_packet=None;source_packet=None;depth_model=None;old_source_packet=None;self_masker=None
    qualifier=QualifiedHazardConsumer();appearance=PastPointRGBMemory();qualified_source=None;appearance_source=None;qualification_rows=[];opportunity=EarlierSourceOpportunity();source_event=None;opportunity_rows=[]
    class CaptureRuntime(runner.IsaacBenchmarkRuntime):
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            result=runner.IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom);return result
        def _capture(self):
            nonlocal current_packet,source_packet,depth_model,old_source_packet,self_masker,qualified_source,appearance_source,source_event
            capture_started=time.perf_counter()
            import omni.usd
            from pxr import UsdGeom
            from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose,_camera_optical_to_base,_get_rgb
            from bev_ground_truth import read_camera_depth
            stage=omni.usd.get_context().get_stage()
            if not getattr(self,'fresh_cameras',False):
                for camera in (self._left_camera,self._right_camera,self._third_camera):camera.set_frequency(-1)
                self.fresh_cameras=True
            def snapshot():
                return [np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            clock=self._clock_metadata();before=snapshot()
            ensure_current_frames((self._left_camera,self._right_camera),clock[2],self._world.render)
            fixed_render_flush(self._world.render,self._clock_metadata,snapshot)
            assert self._clock_metadata()==clock and all(np.array_equal(a,b) for a,b in zip(before,snapshot()))
            visible=marker_visibility(stage,UsdGeom);assert all(v['computed_visibility']=='invisible' for v in visible.values());markers.append(visible)
            step=complete_capture(super()._capture,self._world.render,self._clock_metadata,snapshot,
                                  lambda:(_get_rgb(self._left_camera),_get_rgb(self._right_camera),_get_rgb(self._third_camera)))
            self._captured_frames['stereo_view'][-1]=step.frames['stereo_view'].copy();self._captured_frames['third_person'][-1]=step.frames['third_person'].copy()
            i=len(clocks);t=float(step.simulation_time_s);rawtime=float(step.raw_simulation_time_s)
            eyes={eye:float(cam.get_current_frame()['rendering_time']) for eye,cam in [('left',self._left_camera),('right',self._right_camera)]}
            assert all(abs(rawtime-et)<=1e-5 for et in eyes.values());clocks.append(dict(frame=i,time_s=t,raw_time_s=rawtime,eyes=eyes))
            p,q=self._robot.get_world_pose();execstate=export_execution(self._action_smoother,self._action_protocol) if hasattr(self._action_smoother,'pid') else None;execution_states.append(execstate)
            states.append(dict(time_s=t,position=np.asarray(p).tolist(),quaternion=np.asarray(q).tolist(),velocity=np.asarray(self._robot.get_linear_velocity()).tolist(),angular_velocity=np.asarray(self._robot.get_angular_velocity()).tolist(),joints=np.asarray(self._robot.get_joint_positions()).tolist(),joint_velocity=np.asarray(self._robot.get_joint_velocities()).tolist()))
            contacts.append(dict(frame=i,time_s=t,collision=bool(step.collision),contact=bool(step.physical_contact)))
            rgb={eye+'_rgb':_get_rgb(cam) for eye,cam in [('left',self._left_camera),('right',self._right_camera)]}
            transforms={eye+'_optical_to_base':_camera_optical_to_base(stage,path) for eye,path in [('left',LEFT_CAMERA_PATH),('right',RIGHT_CAMERA_PATH)]}
            poses={eye+'_camera_usd_world':get_cam_pose(stage,path) for eye,path in [('left',LEFT_CAMERA_PATH),('right',RIGHT_CAMERA_PATH)]}
            validate_capture(dict(**rgb,**transforms,raw_time_s=rawtime,left_render_time_s=eyes['left'],right_render_time_s=eyes['right']))
            # Input/pose namespaces cannot accidentally contain offline depth.
            np.savez_compressed(out/'online_inputs'/f'frame_{i:03d}.npz',**rgb,time_s=t)
            np.savez_compressed(out/'known_poses'/f'frame_{i:03d}.npz',**poses,time_s=t)
            np.savez_compressed(out/'history'/f'frame_{i:03d}.npz',proprio_history=step.observation.proprio_history,action_history=step.observation.action_history,quaternion_world_wxyz_history=step.observation.quaternion_world_wxyz_history)
            # RGB source sees only two current image arrays; no renderer depth/pose input.
            point_started=time.perf_counter();predicted=False
            if i%5==0:
                if depth_model is None:depth_model=FastStereo()
                pred=depth_model.predict(rgb['left_rgb'],rgb['right_rgb'],maskdata['left'],maskdata['right'])
                require_boolean_confidence(pred['confidence'])
                np.savez_compressed(out/'source_predictions'/f'frame_{i:03d}.npz',**pred,frame=i,time_s=t)
                if self_masker is None:self_masker=RobotSelfMask()
                old_source_packet,source_packet,current_masks=paired_sources(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(**poses,root_position_world_m=np.asarray(p),time_s=t),step.observation.proprio_history,self_masker,t)
                np.savez_compressed(out/'masks'/f'frame_{i:03d}.npz',**current_masks,current_joints=step.observation.proprio_history[-1,9:12],frame=i,time_s=t)
                np.savez_compressed(out/'old_sensor_packets'/f'source_{i:03d}.npz',**old_source_packet)
                np.savez_compressed(out/'sensor_packets'/f'source_{i:03d}.npz',**source_packet)
                low=len(states)>=5 and all(np.linalg.norm(v['velocity'][:2])<.05 for v in states[-5:])
                count=int(((source_packet['world'][:,2]>.05)&(source_packet['world'][:,2]<=1.5)).sum())
                pair=opportunity.update(source_packet,np.asarray([v['velocity'][:2] for v in states[-5:]]),t,i)
                if pair is not None:source_event=dict(time_s=t,frame=i,confirmed=True,pair=pair)
                qualified=pair is not None
                opportunity_rows.append(dict(frame=i,time_s=t,variant=variant,qualified_seed=bool(qualified),source_event=copy.deepcopy(source_event),opportunity_diagnostics=copy.deepcopy(opportunity.diagnostics),current_diagnostics_valid=bool(opportunity.diagnostics.get('frame')==i)))
                if not appearance.seeded and qualified:appearance.seed(source_packet,dict(**rgb,time_s=t),t,qualified=True)
                proof=appearance.query(dict(**rgb,time_s=t),dict(**poses,time_s=t),current_masks,t)
                qualified_source,appearance_source=qualifier.consume(source_packet,proof,t,qualified)
                np.savez_compressed(out/'qualified_sensor_packets'/f'source_{i:03d}.npz',**qualified_source)
                np.savez_compressed(out/'appearance_old_packets'/f'source_{i:03d}.npz',**appearance_source)
                np.savez_compressed(out/'appearance_predictions'/f'frame_{i:03d}.npz',**proof,frame=i,time_s=t)
                # Causal model-hypothesis outputs sealed BEFORE renderer-depth reads.
                hypothetical=partial_source.update(source_packet,rgb,poses,current_masks,qualified,t,i,np.asarray(p)[:2],newgoal[:2],newstart[:2])
                np.savez_compressed(out/'possible_sets'/f'frame_{i:03d}.npz',**hypothetical)
                for (vf,eye),mask in partial_source.class_masks.items():
                    np.savez_compressed(out/'partial_class_masks'/f'birth_{i:03d}_{eye}_{vf:03d}.npz',potential_hazard_mask=mask,exposure_time_s=vf*.1,hypothesis_birth_time_s=t,source_seed_time_s=partial_source.model['time_s'],not_observed_occupancy=True)
                if partial_source.birth_packet is not None and not (out/'immutable_possible_set_birth.npz').exists():np.savez_compressed(out/'immutable_possible_set_birth.npz',**partial_source.birth_packet)
                if partial_source.model is not None:np.savez_compressed(out/'partial_appearance_model.npz',**partial_source.model)
                qualification_rows.append(dict(frame=i,**qualifier.diagnostics))
                predicted=True
            assert source_packet is not None and source_packet['time_s']<=t
            packet=dict(frame=i,time_s=t,points_world=source_packet['world'],root_xyz=np.asarray(p));current_packet=packet
            clocks[-1].update(rgb_source_wall_s=time.perf_counter()-point_started,source_predicted=predicted,source_frame=int(source_packet['frame']),source_time_s=float(source_packet['time_s']),source_age_s=t-float(source_packet['time_s']))
            assert self._clock_metadata()==clock and all(np.array_equal(a,b) for a,b in zip(before,snapshot()))
            # Ground-truth depths read only AFTER RGB control source has been fixed.
            depths={eye+'_depth_m':read_camera_depth(cam) for eye,cam in [('left',self._left_camera),('right',self._right_camera)]}
            clocks[-1]['capture_partial_wall_s']=time.perf_counter()-capture_started
            np.savez_compressed(out/'offline_truth'/f'frame_{i:03d}.npz',**poses,**transforms,**depths,root_position_world_m=p,root_quaternion_world_wxyz=q,time_s=t)
            if i%30==0:print('CAPTURE',i,t,flush=True)
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=H/'controlled_obstacle_fixture_v1',contract_path=P/'configs/data_contract.yaml',occupancy_path=occupancy,record_video=True,gui=False,diagnostic_physics_trace=True,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',auxiliary_velocity_limit_rad_s=.3))
    protocol=runner.load_action_protocol(P/'configs/data_contract.yaml');policy=PriorRGBPolicy(protocol,memory_mode=arm)
    shadow=ReadOnlyMemoryShadow(protocol) if arm=='current' else ReadOnlyCurrentMemoryShadow(protocol);reset=policy.reset
    def recorded_reset(context):
        reset(context);shadow.reset(context)
    policy.reset=recorded_reset
    gate=SafeStartBootstrap();act=policy.act
    def recorded_act(observation,goal):
        i=len(policies)
        if policy.grid is None:
            policy.grid=StaticSeedBeliefGrid(np.asarray(observation.position_world_xy)-20) if variant=='C1' else QualifiedHazardGrid(np.asarray(observation.position_world_xy)-20,memory_mode=arm)
        if variant=='C1' and qualifier.seed is not None and policy.grid.belief_seed is None:policy.grid.seed(qualifier.seed,float(current_packet['time_s']))
        if variant=='B0':policy.grid.set_memory_source(None)
        policy.set_source_packet(qualified_source);policy.set_current_packet(current_packet);begin=time.perf_counter();action=act(observation,goal);policy_wall_s=time.perf_counter()-begin
        nominal=action.copy()
        # Source-only acquisition: do not dispatch nominal planner or gate output.
        gate.choose(policy.records[-1]['time_s'],float(np.linalg.norm(observation.proprio_history[-1,:2])),source_packet,nominal,source_event,i)
        action=np.zeros(10,np.float32)
        policy.records[-1].update(nominal_request=nominal.tolist(),request=action.tolist(),execution_role='source_only_zero_requests_no_navigation',nominal_policy_fields_not_applied=bool(np.any(action!=nominal)),source_gate_ready=gate.handed_off,observation_handed_off=False,observation_low_count=gate.low_count,source_readiness_time_s=gate.handoff_time_s,observation_handoff_time_s=None,source_gate_variant=variant,source_event_frame=source_event['frame'] if source_event else None,source_event_time_s=source_event['time_s'] if source_event else None)
        before_shadow=copy.deepcopy(boundary_state(policy.tracker));action_copy=action.copy();clock_before=runtime._clock_metadata()
        physical_before=[np.asarray(v).copy() for v in (*runtime._robot.get_world_pose(),runtime._robot.get_linear_velocity(),runtime._robot.get_angular_velocity(),runtime._robot.get_joint_positions(),runtime._robot.get_joint_velocities())]
        shadow_started=time.perf_counter()
        shadow.consume(source_packet,current_packet,observation,goal);shadow.records[-1].update(shadow_wall_s=time.perf_counter()-shadow_started,source_filter='dynamic_current_joint_CAD',memory_role='history' if arm=='current' else 'current')
        physical_after=[np.asarray(v).copy() for v in (*runtime._robot.get_world_pose(),runtime._robot.get_linear_velocity(),runtime._robot.get_angular_velocity(),runtime._robot.get_joint_positions(),runtime._robot.get_joint_velocities())]
        assert boundary_state(policy.tracker)==before_shadow and np.array_equal(action,action_copy) and runtime._clock_metadata()==clock_before and all(np.array_equal(a,b) for a,b in zip(physical_before,physical_after))
        np.savez_compressed(out/'policy_inputs'/f'frame_{i:03d}.npz',left_rgb=observation.stereo_left_rgb,right_rgb=observation.stereo_right_rgb,proprio_history=observation.proprio_history,action_history=observation.action_history,quaternion_history=observation.quaternion_world_wxyz_history,position_world_xy=observation.position_world_xy,yaw_world_rad=observation.yaw_world_rad,pointgoal_base_xy=goal,request=action)
        policies.append(dict(frame=i,diagnostics=policy.diagnostics,request=policy.request_diagnostics,tracking=policy.tracking_diagnostics,mapping=policy.tracker.mapping_records[-1] if policy.tracker.mapping_records else None,local=policy.records[-1],policy_wall_s=policy_wall_s))
        grid=policy.grid
        belief_seed=getattr(grid,'belief_seed',None)
        if belief_seed is not None and not (out/'immutable_belief_seed.npz').exists():np.savez_compressed(out/'immutable_belief_seed.npz',**belief_seed)
        np.savez_compressed(out/'maps'/f'frame_{i:03d}.npz',labels=grid.labels,provenance=grid.provenance(),floor_time=grid.memory.floor_time,hazard_time=grid.memory.hazard_time,origin=grid.origin,resolution=grid.resolution,time_s=grid.time,task_ground_prior=grid.task_ground_prior,safe_start_prior=grid.safe_start_prior,task_prior_is_observation=False,static_belief_observation_time=getattr(grid,'belief_observation_time',np.full(grid.labels.shape,-np.inf)),static_belief_is_observation=False)
        if policy.plan is not None:
            way=policy.plan.waypoints
            np.savez_compressed(out/'local_plans'/f'frame_{i:03d}.npz',pose=np.array([[p.x_m,p.y_m,p.yaw_rad] for p in way]),direction=np.array([1 if p.drive_direction=='forward' else -1 for p in way]),command=np.array([[p.main_command_rad_s,p.auxiliary_angle_rad] for p in way]),time_s=policy.last_plan,version=policy.local_reference_version)
        return action
    policy.act=recorded_act
    result=None;code=1
    try:
        result=run_episode(runtime,policy,episode,action_protocol=protocol,config=RunnerConfig(episode_budget_s=8.))
        arrays={f.name:getattr(result,f.name) for f in dataclasses.fields(result) if isinstance(getattr(result,f.name),np.ndarray)}
        np.savez_compressed(out/'episode_arrays.npz',**arrays)
        (out/'metrics.json').write_text(json.dumps(result.to_metrics_dict(),indent=2)+'\n')
        np.savez_compressed(out/'oracle_path.npz',xy=runtime.oracle_path_xy(episode))
        occ=runtime.occupancy;np.savez_compressed(out/'offline_occupancy_label.npz',labels=occ.grid,origin=occ.global_origin_xy,resolution=occ.resolution_m)
        code=0 if result.runtime_error is None else 1
        print('CONTROLLED_HEADING_SOURCE_PROBE_COMPLETE',arm,result.termination_reason,result.elapsed_s,result.final_distance_m,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        for name,data in [('camera_clocks',clocks),('states',states),('execution_states',execution_states),('contacts',contacts),('marker_checks',markers),('physics_contacts',runtime.collision_contact_samples),('policy_records',policies),('shadow_records',shadow.records),('hazard_qualification_records',qualification_rows),('source_opportunity_records',opportunity_rows)]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(completed=code==0,policy_calls=len(policies),captures=len(clocks),reset_count=1,internal_reset=False,scene_sha256=episode.scene_sha256,occupancy_sha256=episode.occupancy_sha256,camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',controller_class=str(type(policy)),runtime_class=str(type(runtime)),source_sha256={f:sha(f) for f in config['source_sha256']},global_reference_consumed=False,rgb_depth_knownpose=True,source_model_admitted_in_old_domain=False,task_prior_is_observation=False,local_goal_m=4.4,new_task_budget_s=8.,source_only=True,navigation_executed=False,initial_yaw_deg=0.,original_formal_budget_s=35.,fixture_not_original_E1=True,shared_zero_bootstrap=True,static_persistence_prior=(variant=='C1'),belief_code=5,belief_not_observation=True,bootstrap_blocked=gate.blocked,control_variant=variant,observation_time_counts=True,source_readiness_time_s=gate.handoff_time_s,observation_handoff_time_s=None,memory_arm=arm,dynamic_selfmask=True,same_masked_source_opposite_memory_shadow=True,primary_memory=arm,shadow_memory='history' if arm=='current' else 'current',shadow_executed=False,shared_unknown_request_main_cap_rad_s=.3,formal_metrics_not_applicable=True,source_model_admitted=False,nominal_geometry_only=True,extra_physics_tail=False,hazard_source_qualification=True,estimated_pose_not_admitted=True,appearance_geometry_source_time_not_refreshed=True,source_event_handoff_shared=True,gate_class=str(type(gate)),gate_choose_source_file=inspect.getsourcefile(gate.choose),policy_act_qualname=act.__qualname__,policy_act_source_file=inspect.getsourcefile(act),policy_act_source_sha256=sha(inspect.getsourcefile(act))),indent=2)+'\n')
        manifest={str(p):sha(p) for p in out.rglob('*') if p.is_file()}
        (out/'artifact_manifest.json').write_text(json.dumps(dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact_sha256=manifest,source_model_admitted=False,rgb_control=True,observation_scheduled_prefix=False,shared_safe_zero_bootstrap=True,control_variant='initial_heading_source_only',knownpose_privileged=True,task_prior_is_observation=False,not_formal_navigation=True,manifest_scope="pre-runtime-close; separate post-close manifest is authoritative"),indent=2)+'\n')
        runtime.close(return_code=code)
if __name__=='__main__':main()
