"""ONE supplied request-program physical arm; NOT B0 tracker or navigation; no NN."""
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
from typed_hypothesis_grid import TypedHypothesisGrid,require_bootstrap_progress
from execution_domain_four_point_consumer_v2 import TrackingTubeGrid,with_supported_four_points as with_tracking_tube
from execution_domain_four_point_source_v2 import source_identity as primitive_source_identity
from command_domain_cap import with_command_domain_cap
from native_rolling_shadow import NativeRollingShadow as OnlineRollingShadow
from imu_interval_packet import interval_packet
from flat_ground_pose_bridge import FlatGroundPoseBridge
from bank_rock_observation_schedule import request as observation_request
from provided_historical_model import installable_model
from bank_hold_request_program import BankHoldRequestProgram
from roll_pid import bank_from_quaternion
from bilateral_core_memory import BilateralCoreMemory
from persistent_core_grid import PersistentCoreGrid
from ray_contradiction_memory import contradict_cores
from partial_cue_source_v2 import inflate_xy
from probe_cleanup import install_sigterm_cleanup,export_then_close
from observation_clock_adapter import observe_clock_without_act
from estimated_navigation_observation import EstimatedNavigationObservation
from scipy.spatial.transform import Rotation
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
    config=json.loads((H/'bank_hold_request_program_protocol_v1.json').read_text())
    for f,s in config['source_sha256'].items():assert sha(f)==s,f
    prior_seed_path=Path(config['provided_prior_seed']);assert sha(prior_seed_path)==config['input_sha256'][str(prior_seed_path)];prior_raw=dict(np.load(prior_seed_path))
    variant=sys.argv[1];assert variant in config['variants'];arm='current';out=H/('bank_hold_request_program_'+variant+'_run_v1');out.mkdir(exist_ok=False)
    for name in ['estimated_poses','online_inputs','known_poses','offline_truth','history','sensor_packets','source_predictions','maps','local_plans','policy_inputs','masks','old_sensor_packets','qualified_sensor_packets','appearance_old_packets','appearance_predictions','possible_sets','partial_class_masks']:(out/name).mkdir()
    episode=next(e for e in runner.load_manifest(config['manifest']).episodes if e.episode_id==config['episode_id'])
    occupancy=H/'controlled_obstacle_fixture_v1/occupancy.npz'
    original_episode=episode
    newstart=np.array([-2.2,.15,.45]);newgoal=np.array([2.2,.15,.45])
    episode=dataclasses.replace(episode,episode_id='controlled_static_obstacle_v1',scene_uri='${SCENE_ROOT}/single_cylinder.usda',scene_sha256=sha(H/'controlled_obstacle_fixture_v1/single_cylinder.usda'),occupancy_sha256=sha(occupancy),start_quaternion_world_wxyz=(1.,0.,0.,0.),start_position_world_m=tuple(newstart),goal_position_world_m=tuple(newgoal),euclidean_distance_m=4.4)
    (out/'task_episode.json').write_text(json.dumps(dict(task='given_request_program_physical60s_not_navigation',original_episode=original_episode.to_dict(),actual_episode=episode.to_dict(),not_original_complete_E1=True,oracle_path_metadata_inherited_not_recomputed=True,SPL_not_used_for_this_diagnostic=True),indent=2)+'\n')
    validate_mask_identity(json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text()))
    maskdata=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    yy,xx=np.mgrid[0:480:4,0:640:4]
    rays=np.column_stack(((xx.ravel()-319.5)/584.8888888888889,(yy.ravel()-239.5)/584.8888888888889,np.ones(xx.size)))
    clocks=[];states=[];execution_states=[];contacts=[];markers=[];policies=[];current_packet=None;source_packet=None;depth_model=None;old_source_packet=None;self_masker=None
    qualifier=QualifiedHazardConsumer();appearance=PastPointRGBMemory();qualified_source=None;appearance_source=None;qualification_rows=[];opportunity=EarlierSourceOpportunity();source_event=None;opportunity_rows=[]
    rolling=OnlineRollingShadow();pose_bridge=None;navigation_observation=EstimatedNavigationObservation();estimated_states=[];native_reads=[];native_interface=None;native_path=None;native_creation=None;core_memory=BilateralCoreMemory();ray_pairs=[];memory_packet=None;initial_estimated_xy=None;stop_low_count=0
    class CaptureRuntime(runner.IsaacBenchmarkRuntime):
        def _configured_articulation_gains(self):
            gains=super()._configured_articulation_gains()
            if not hasattr(self,'_interval_original_step'):
                self._interval_original_step=self._world.step
                def measured_step(*args,**kwargs):
                    begin=float(self._world.current_time);result=self._interval_original_step(*args,**kwargs);self._actual_last_interval=(begin,float(self._world.current_time));return result
                self._world.step=measured_step
            return gains
        def _record_physics_diagnostic(self,*args,**kwargs):
            super()._record_physics_diagnostic(*args,**kwargs)
            before={k:np.asarray(v[-1]).copy() for k,v in self._physics_diagnostics.items()};clock=self._clock_metadata()
            reading=native_interface.get_sensor_reading(native_path,use_latest_data=True,read_gravity=True);begin,end=self._actual_last_interval
            assert abs(end-self._physics_diagnostics['raw_simulation_time_s'][-1])<=1e-10
            sensor,diagnostic=interval_packet(reading,begin,end,self._physics_diagnostics['simulation_time_s'][-1],self._physics_diagnostics['joint_velocity_rad_s'][-1][0]);native_reads.append(diagnostic)
            if sensor is None:raise RuntimeError('native qualification failed; noideal fallback '+str(diagnostic['reason']))
            normal=None
            if rolling.model is None:
                q0=np.asarray(self._physics_diagnostics['root_quaternion_world_wxyz'][0]);normal=Rotation.from_quat(q0[[1,2,3,0]]).as_matrix().T@np.array([0.,0.,1.])
            assert rolling.consume(sensor,normal) is None
            assert self._clock_metadata()==clock and all(np.array_equal(before[k],np.asarray(v[-1])) for k,v in self._physics_diagnostics.items())
            if pose_bridge is not None:pose_bridge.consume(rolling.records[-1])

        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            result=runner.IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom)
            nonlocal native_interface,native_path,native_creation
            import omni.kit.commands
            from isaacsim.sensors.physics import _sensor
            from collect_data_dubins import BASE_ROOT
            native_path=BASE_ROOT+'/research_native_imu'
            ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/research_native_imu',parent=BASE_ROOT,sensor_period=.02,translation=gf.Vec3d(0,0,0),orientation=gf.Quatd(1,0,0,0),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1)
            assert ok and prim is not None
            native_interface=_sensor.acquire_imu_sensor_interface();native_creation=dict(path=native_path,period_s=.02,filter_sizes=[1,1,1],orientation_field_read=False,read_gravity=True,use_latest_data=True,lifecycle='beforeexistingformalreset',extra_reset=False)
            return result
        def _capture(self):
            nonlocal current_packet,source_packet,depth_model,old_source_packet,self_masker,qualified_source,appearance_source,source_event,pose_bridge,memory_packet,initial_estimated_xy
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
            if pose_bridge is None:
                assert i==0
                initial_R=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix();pose_bridge=FlatGroundPoseBridge(p,initial_R)
                for record in rolling.records:pose_bridge.consume(record)
                (out/'initial_gauge.json').write_text(json.dumps(dict(position=np.asarray(p).tolist(),rotation=initial_R.tolist(),privileged_initial_only=True,floor_z_prior_m=0.,CAD_radius_prior_m=.4000000059604645,height_prior_is_observation=False),indent=2)+'\n')
            estimated_pose=pose_bridge.at(t);estimated_states.append(estimated_pose)
            if i==0:initial_estimated_xy=estimated_pose["root_position_world_m"][:2].copy()
            np.savez_compressed(out/'estimated_poses'/f'frame_{i:03d}.npz',**estimated_pose)
            source_poses={k:estimated_pose[k] for k in ('left_camera_usd_world','right_camera_usd_world')}
            # RGB source sees only RGB plus estimated exposure pose; current GT is archive-only.

            source_proprio=step.observation.proprio_history.copy();source_proprio[:,:9]=0.
            point_started=time.perf_counter();predicted=False
            # This capture contributes a clock only. Geometry is a separately
            # provided historical MODEL, never a current RGB/depth observation.
            source_packet=dict(frame=i,time_s=t,world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int),source_consumer='CURRENT_CLOCK_ONLY_NO_GEOMETRY',current_observation=False)
            np.savez_compressed(out/'sensor_packets'/f'source_{i:03d}.npz',**source_packet)
            if i==140:
                memory_packet=installable_model(prior_raw,t,prior_sequence=config['provided_prior_sequence'],prior_sha256=sha(prior_seed_path))
                provenance=memory_packet.pop('provided_model_provenance')
                (out/'provided_model_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
                np.savez_compressed(out/'frozen_memory_seed.npz',**memory_packet)
                for key in ['possible_xy','raw_world_voxel_centres']:np.testing.assert_array_equal(memory_packet[key],prior_raw[key])
                (out/'memory_actor_pretruth_seal.json').write_text(json.dumps({str(out/n):sha(out/n) for n in ['provided_model_provenance.json','frozen_memory_seed.npz']},indent=2)+'\n')
            assert source_packet is not None and source_packet['time_s']<=t
            packet=dict(frame=i,time_s=t,points_world=np.empty((0,3)),root_xyz=estimated_pose["root_position_world_m"].copy());current_packet=packet
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
    protocol=runner.load_action_protocol(P/'configs/data_contract.yaml')
    class RequestProgramPolicy:
        def reset(self,context):pass
        def act(self,observation,goal):raise RuntimeError('only installed given-program action entry')
    policy=RequestProgramPolicy()
    given=json.loads(Path(config['given_request_program']).read_text());program=BankHoldRequestProgram(given['requests'],given['modes'],variant)
    shadow=ReadOnlyMemoryShadow(protocol) if arm=='current' else ReadOnlyCurrentMemoryShadow(protocol);reset=policy.reset
    def recorded_reset(context):
        reset(context);shadow.reset(context)
    policy.reset=recorded_reset
    def recorded_act(observation,goal):
        i=len(policies);t=float(current_packet['time_s']);pose=pose_bridge.at(t)
        native_q=Rotation.from_matrix(pose['root_rotation_world']).as_quat()
        current_bank=bank_from_quaternion(native_q[[3,0,1,2]])
        action=program.act(i,current_bank,float(pose['time_s']),t)
        rec=copy.deepcopy(program.records[-1]);rec.update(planner_act_called=False,RGB_or_map_consumed=False,current_tracker_not_instantiated=True,request_program_only=True,source_model_admitted=False)
        policies.append(dict(frame=i,time_s=t,request=action.tolist(),planner_act_called=False,local=rec));return action
    act=recorded_act
    policy.act=recorded_act
    result=None;code=1
    install_sigterm_cleanup()
    try:
        result=run_episode(runtime,policy,episode,action_protocol=protocol,config=RunnerConfig(episode_budget_s=60.))
        arrays={f.name:getattr(result,f.name) for f in dataclasses.fields(result) if isinstance(getattr(result,f.name),np.ndarray)}
        np.savez_compressed(out/'episode_arrays.npz',**arrays)
        (out/'metrics.json').write_text(json.dumps(result.to_metrics_dict(),indent=2)+'\n')
        np.savez_compressed(out/'oracle_path.npz',xy=runtime.oracle_path_xy(episode))
        occ=runtime.occupancy;np.savez_compressed(out/'offline_occupancy_label.npz',labels=occ.grid,origin=occ.global_origin_xy,resolution=occ.resolution_m)
        code=0 if result.runtime_error is None else 1
        print('GIVEN_REQUEST_PROGRAM_COMPLETE',arm,result.termination_reason,result.elapsed_s,result.final_distance_m,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        def export_evidence():
            (out/'native_imu_reads.json').write_text(json.dumps(native_reads,indent=2)+'\n')
            (out/'native_imu_identity.json').write_text(json.dumps(dict(creation=native_creation,initial_normal=None if rolling.initial_normal is None else rolling.initial_normal.tolist(),orientation_field_read=False,pose_consumed_by_control=(variant=='C')),indent=2)+'\n')
            np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
            for name,frames in runtime.captured_frames.items():write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
            for name,data in [('native_rolling_sensors',rolling.sensors),('rolling_predictions',rolling.records),('camera_clocks',clocks),('states',states),('execution_states',execution_states),('contacts',contacts),('marker_checks',markers),('physics_contacts',runtime.collision_contact_samples),('policy_records',policies),('shadow_records',shadow.records),('hazard_qualification_records',qualification_rows),('source_opportunity_records',opportunity_rows)]:
                (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
            (out/'identity.json').write_text(json.dumps(dict(completed=code==0,policy_calls=len(policies),captures=len(clocks),reset_count=1,internal_reset=False,camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',source_sha256=config['source_sha256'],source_only=False,navigation_executed=False,given_request_program_executed=True,actual_actor='given_request_program_current_native_bank_hold_'+variant,RGB_partial_hypothesis_estimatedpose=False,live_RGB_geometry_consumer=False,provided_historical_model_not_used_by_actor=True,no_NN_inference=True,native_IMU_pose_provider=True,ideal_bodygyro_privileged=False,lowlevel_rollPID_ideal_IMU_privileged=True,initial_gauge_privileged=True,flat_ground_height_prior=True,height_prior_is_observation=False,controller_class=str(type(policy)),policy_class_mro=[c.__module__+'.'+c.__qualname__ for c in type(policy).__mro__],policy_act_source=inspect.getsourcefile(act),policy_act_sha256=sha(inspect.getsourcefile(act)),start_goal_y_offset_m=.15,old_program_observation_annotation_s=14.,episode_budget_s=60.,original_formal_budget_s=35.,formal_metrics_not_applicable=True,source_model_admitted=False,measured_geometry_consumed=False,moving_geometry_consumed=False,historical_RGB_class_model_not_instantiated=True,provided_model_geometry_frozen_after_installation=True,provided_model_installed_at14_not_exposure=True,source_consumer_variant=variant,only_pose_provider_component_changed=False,only_source_provider_changed_relative_parent=False,request_program_not_closed_loop_B0=True,primitive_source_identity=primitive_source_identity(),no_navigation_policy_or_shadow_ever=True,given_program_act_from_frame0=True,model_persistence_is_prior=True,negative_ray_veto_not_observed_free=True,extra_physics_tail=False),indent=2)+'\n')
            manifest={str(p):sha(p) for p in out.rglob('*') if p.is_file()}
            (out/'artifact_manifest.json').write_text(json.dumps(dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact_sha256=manifest,source_model_admitted=False,rgb_control=False,provided_historical_model_not_used_by_actor=True,no_NN_inference=True,observation_scheduled_prefix=True,shared_safe_zero_bootstrap=True,control_variant='given_request_program_'+variant,knownpose_privileged=False,native_IMU_pose_provider=True,ideal_bodygyro_privileged=False,lowlevel_rollPID_ideal_IMU_privileged=True,initial_gauge_privileged=True,task_prior_is_observation=False,not_formal_navigation=True,manifest_scope="pre-runtime-close; separate post-close manifest is authoritative"),indent=2)+'\n')
        export_then_close(runtime,export_evidence,code)
if __name__=='__main__':main()
