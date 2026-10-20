"""Frozen sparse SGBM E1 pair; one projected-heading-rate feedback consumer."""
from pathlib import Path as _BootPath
import sys as _BootSys
_BootSys.path.insert(0,str(_BootPath(__file__).resolve().parent.parent/'stereo_memory_navigation_20261004_v1'))
from heading_rate import with_heading_rate_policy
import argparse,hashlib,json,sys,traceback,dataclasses,time
from pathlib import Path
import numpy as np
import cv2
from stereo_geometry import SGBMStereo
from masked_source_pair import paired_sources as sparse_paired_sources
from dense_current_live_transport_v1 import paired_sources as dense_paired_sources
from timed_static_base_self_mask import TimedStaticBaseSelfMask as RobotSelfMask
from stereo_geometry import restore_native
from paired_texture_surface import PairedTextureSurfaceMemory
from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from nonbinding_side_hypothesis_v1 import NonbindingSideHypothesisGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from readonly_boundary_shadow_v1 import ReadOnlyBoundaryShadow
from retained_reference_tube_viability_v1 import with_tube_viability_after_measured_reversal
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from policy_row_journal_v1 import PolicyRowJournal,recover_to_files
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from estimated_navigation_observation import EstimatedNavigationObservation
from tentative_point_memory import TentativePointMemory
from standby_handoff_v1 import StandbyHandoff as FrontViewAcquisition
from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
from front_view_baseline_shadow_v1 import with_front_view_baseline_shadow
from observation_clock_adapter import observe_clock_without_act
from guarded_reference import boundary_state
from benchmark.policies import EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from imu_gravity_rolling import GravityRolling
from imu_interval_packet import interval_packet
from scipy.spatial.transform import Rotation
from execution_snapshot import export_execution
from probe_cleanup import install_sigterm_cleanup,export_then_close
from scripts.camera_rig import camera_rig_metadata
from render_flush import fixed_render_flush
from capture_transaction import replace_step_images
from benchmark.artifacts import write_rgb_video
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import install_runtime
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from render_boundary import hide_diagnostic_markers,marker_visibility
sys.path.insert(0,str(H.parent/'clean_visual_hard0_intervention_20261003_v1'))
from clock_sync import ensure_current_frames
sys.path.insert(0,str(H.parent/'visible_surface_connectivity_20261004_v1'))
from rig_inputs import validate_capture

def main():
    install_sigterm_cleanup()
    parser=argparse.ArgumentParser();parser.add_argument('--arm',choices=['b0','candidate'],required=True);a=parser.parse_args();a.condition='e1';a.name='heading_rate_e1_'+a.arm
    paired_sources=sparse_paired_sources
    config=json.loads((H/('heading_rate_e1_'+a.arm+'_protocol_v3.json')).read_text())
    for file,digest in config['source_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    for file,digest in config['input_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    out=H/('heading_rate_e1_'+a.arm+'_run_v2');out.mkdir(parents=True,exist_ok=False)
    (out/'live_transport_contract.json').write_text(json.dumps(dict(arm=a.arm,protocol=config),indent=2)+'\n')
    (out/'frames').mkdir()
    for n in ['source_predictions','sensor_packets','known_poses','partial_observations','source_inputs','baseline_partial_observations','maps','local_plans','raw_point_memory_snapshots','current_masks']:(out/n).mkdir()
    extrinsics=np.load(config['rig_extrinsics'])
    static_masks=np.load(config['static_masks']);surface_memory=PairedTextureSurfaceMemory();baseline_surface_memory=PairedTextureSurfaceMemory();baseline_point_memory=TentativePointMemory();raw_point_memory=TentativePointMemory();depth_model=None;self_masker=None;source_rows=[]
    actual_requests=[]
    from benchmark.manifest import BenchmarkEpisode
    manifest=json.loads(Path(config['episode_manifest']).read_text())
    episode=BenchmarkEpisode.from_dict(next(e for e in manifest['episodes'] if e['episode_id']==config['episode_id']))
    (out/'task_episode.json').write_text(json.dumps(dict(actual_task='frozen_classical_SGBM_binocular_memory_rollPID_knownpose_TaskH_35s',static_scene_assumption=True,only_control_change='projected_heading_rate_feedback' if a.arm=='candidate' else 'none_B0',only_source_change='none_original_sparse_both_arms',original_time_rule_s=35.,strict_visual_admission=False,comparison='knownpose_TaskH_E1_projected_heading_rate_feedback_twoarm_not_fullcausal',actual_episode=episode.to_dict(),original_episode_preserved=True,not_navigation=False,not_original35=False,original35_time_budget=True,original_strict_cold_start_task=False,SPL_applicable=False,planner_feedback_current_only=True,current_GT_pose_registration=True),indent=2)+'\n')
    clocks=[];states=[];imu_reads=[];imu_packets=[];imu_predictions=[];imu_model=None;imu_initial_normal=None;imu_interface=None;imu_path=None;imu_creation=None;gauge=None

    class CaptureRuntime(IsaacBenchmarkRuntime):
        def _configured_articulation_gains(self):
            gains=super()._configured_articulation_gains()
            if not hasattr(self,'_interval_original_step'):
                self._interval_original_step=self._world.step
                def measured_step(*args,**kwargs):
                    begin=float(self._world.current_time);result=self._interval_original_step(*args,**kwargs)
                    self._actual_last_interval=(begin,float(self._world.current_time));return result
                self._world.step=measured_step
            return gains
        def _record_physics_diagnostic(self,*args,**kwargs):
            nonlocal imu_model,imu_initial_normal,gauge
            super()._record_physics_diagnostic(*args,**kwargs)
            if gauge is None:
                R0=Rotation.from_quat(np.asarray(self._physics_diagnostics['root_quaternion_world_wxyz'][0])[[1,2,3,0]]).as_matrix()
                p0=np.asarray(self._physics_diagnostics['root_position_world_m'][0]).copy()
                imu_initial_normal=R0.T@np.array([0.,0.,1.]);gauge=dict(position=p0.tolist(),rotation=R0.tolist(),initial_only=True,privileged=True)
            reading=imu_interface.get_sensor_reading(imu_path,use_latest_data=True,read_gravity=True)
            begin,end=self._actual_last_interval
            assert abs(end-self._physics_diagnostics['raw_simulation_time_s'][-1])<=1e-10
            packet,diagnostic=interval_packet(reading,begin,end,self._physics_diagnostics['simulation_time_s'][-1],self._physics_diagnostics['joint_velocity_rad_s'][-1][0])
            imu_reads.append(diagnostic);imu_packets.append(None if packet is None else {k:np.asarray(v).tolist() for k,v in packet.items()})
            if packet is None:imu_predictions.append(dict(time_s=diagnostic['time_s'],available=False,reason=diagnostic['reason']))
            else:
                if imu_model is None:imu_model=GravityRolling(imu_initial_normal)
                result=imu_model.update(packet);R0=np.asarray(gauge['rotation']);world=np.asarray(gauge['position'])+R0@result['position'];world[2]=.4000000059604645
                imu_predictions.append(dict(time_s=diagnostic['time_s'],available=True,relative_position=result['position'].tolist(),relative_rotation=result['rotation'].tolist(),position_world=world.tolist(),rotation_world=(R0@result['rotation']).tolist(),velocity_world_m_s=(R0@result['velocity']).tolist(),gravity_used=bool(result['gravity_used']),live_before_next_action=True,flat_height_is_prior=True))

        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            r=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom)
            nonlocal imu_interface,imu_path,imu_creation
            import omni.kit.commands
            from isaacsim.sensors.physics import _sensor
            from collect_data_dubins import BASE_ROOT
            imu_path=BASE_ROOT+'/research_native_imu'
            ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/research_native_imu',parent=BASE_ROOT,sensor_period=.02,translation=gf.Vec3d(0,0,0),orientation=gf.Quatd(1,0,0,0),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1)
            assert ok and prim is not None
            imu_interface=_sensor.acquire_imu_sensor_interface();imu_creation=dict(path=imu_path,period_s=.02,filter_sizes=[1,1,1],orientation_field_read=False,read_gravity=True,use_latest_data=True,lifecycle='existing marker hook before formal reset',extra_reset=False)
            return r
        def _capture(self):
            nonlocal depth_model,self_masker
            import omni.usd
            from pxr import UsdGeom
            from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose,_camera_optical_to_base,_get_rgb
            from bev_ground_truth import read_camera_depth
            stage=omni.usd.get_context().get_stage()
            if not getattr(self,'fresh_cameras',False):
                for camera in (self._left_camera,self._right_camera,self._third_camera):camera.set_frequency(-1)
                self.fresh_cameras=True
            clock=self._clock_metadata()
            before=[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            ensure_current_frames((self._left_camera,self._right_camera),clock[2],self._world.render)
            after=[np.asarray(v) for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            assert self._clock_metadata()==clock and all(np.array_equal(x,y) for x,y in zip(before,after))
            markers=marker_visibility(stage,UsdGeom);assert all(v['computed_visibility']=='invisible' for v in markers.values())
            def snapshot():
                return [np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            fixed_render_flush(self._world.render,self._clock_metadata,snapshot)
            step=super()._capture()
            fixed_render_flush(self._world.render,self._clock_metadata,snapshot)
            step=replace_step_images(step,_get_rgb(self._left_camera),_get_rgb(self._right_camera),_get_rgb(self._third_camera))
            self._captured_frames['stereo_view'][-1]=step.frames['stereo_view'].copy()
            self._captured_frames['third_person'][-1]=step.frames['third_person'].copy()
            i=len(clocks)
            execution=out/'execution';execution.mkdir(exist_ok=True)
            (execution/f'frame_{i:03d}.json').write_text(json.dumps(export_execution(self._action_smoother,self._action_protocol) if hasattr(self._action_smoother,'pid') else None,indent=2)+'\n')
            history=out/'history';history.mkdir(exist_ok=True)
            np.savez_compressed(history/f'frame_{i:03d}.npz',proprio_history=step.observation.proprio_history,action_history=step.observation.action_history,quaternion_world_wxyz_history=step.observation.quaternion_world_wxyz_history)
            times={eye:float(camera.get_current_frame()['rendering_time']) for eye,camera in [('left',self._left_camera),('right',self._right_camera)]}
            assert all(abs(float(step.raw_simulation_time_s)-t)<=1e-5 for t in times.values())
            clocks.append(dict(frame=i,raw_time_s=float(step.raw_simulation_time_s),episode_time_s=float(step.simulation_time_s),eyes=times,markers_hidden=True))
            position,q=self._robot.get_world_pose()
            states.append(dict(time_s=float(step.simulation_time_s),position=np.asarray(position).tolist(),quaternion=np.asarray(q).tolist(),velocity=np.asarray(self._robot.get_linear_velocity()).tolist(),joints=np.asarray(self._robot.get_joint_positions()).tolist()))
            if i%5==0:
                rgb={eye+'_rgb':_get_rgb(camera) for eye,camera in [('left',self._left_camera),('right',self._right_camera)]}
                transforms={eye+'_optical_to_base':_camera_optical_to_base(stage,path) for eye,path in [('left',LEFT_CAMERA_PATH),('right',RIGHT_CAMERA_PATH)]}
                validate_capture(dict(**rgb,**transforms,raw_time_s=float(step.raw_simulation_time_s),left_render_time_s=times['left'],right_render_time_s=times['right']))
                # Explicit current GT pose upperbound, never GT depth for source selection.
                native=imu_predictions[-1];assert native['available'] and abs(native['time_s']-float(step.simulation_time_s))<1e-8
                source_poses={eye+'_camera_usd_world':get_cam_pose(stage,path) for eye,path in [('left',LEFT_CAMERA_PATH),('right',RIGHT_CAMERA_PATH)]}
                rootT=np.eye(4);rootT[:3,3]=position
                residuals={eye:float(np.max(np.abs(transforms[eye+'_optical_to_base']-extrinsics[eye+'_optical_to_base']))) for eye in ('left','right')}
                t=float(step.simulation_time_s)
                np.savez_compressed(out/'source_inputs'/f'frame_{i:03d}.npz',**rgb,time_s=t)
                np.savez_compressed(out/'known_poses'/f'frame_{i:03d}.npz',**source_poses,root_position_world_m=rootT[:3,3],time_s=t)
                if depth_model is None:depth_model=SGBMStereo()
                pred=depth_model.predict(rgb['left_rgb'],rgb['right_rgb'],static_masks['left'],static_masks['right'])
                np.savez_compressed(out/'source_predictions'/f'frame_{i:03d}.npz',**pred,frame=i,time_s=t)
                if self_masker is None:self_masker=RobotSelfMask()
                proprio=step.observation.proprio_history.copy();proprio[:,:9]=0.
                _,source_packet,current_masks=paired_sources(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(**source_poses,root_position_world_m=rootT[:3,3],time_s=t),proprio,self_masker,t)
                np.savez_compressed(out/'sensor_packets'/f'source_{i:03d}.npz',**source_packet)
                np.savez_compressed(out/'current_masks'/f'frame_{i:03d}.npz',left=current_masks['left'],right=current_masks['right'],time_s=t,frame=i,joints_current=proprio[-1,9:12])
                sourcefiles=[out/n/f'frame_{i:03d}.npz' for n in ('source_inputs','known_poses','source_predictions')]+[out/'sensor_packets'/f'source_{i:03d}.npz',out/'current_masks'/f'frame_{i:03d}.npz']
                hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sourcefiles}
                surface=surface_memory.update(source_packet,rgb['left_rgb'],rgb['right_rgb'],t,hashes,restore_native(pred['disparity_px']))
                baseline_surface=baseline_surface_memory.update(source_packet,rgb['left_rgb'],rgb['right_rgb'],t,hashes,restore_native(pred['disparity_px']))
                np.savez_compressed(out/'baseline_partial_observations'/f'frame_{i:03d}.npz',**{k:v for k,v in baseline_surface.items() if k!='input_hashes'})
                baseline_point_memory.update(dict(frame=i,time_s=t,world=baseline_surface['world'],query_pixel_id=baseline_surface['query_pixel_id'],partner_pixel_id=baseline_surface['partner_pixel_id'],input_sha256=hashes),t)
                np.savez_compressed(out/'partial_observations'/f'frame_{i:03d}.npz',**{k:v for k,v in surface.items() if k!='input_hashes'})
                source_rows.append(dict(frame=i,time_s=t,points=len(surface['world']),source_packet_points=len(source_packet['world']),live_packet_sampler='original_stride2',actual_source_transport_module=paired_sources.__module__,CAD_projection_wall_s=self_masker.last_project_wall_s,provenance=surface['input_hashes'],memory_acquisition_times_s=[v['acquisition_time_s'] for v in surface_memory.at(t)],depth_truth_read_after_output=True,current_GT_pose_registration=True,rig_residual_max_abs=residuals))
                raw_point_memory.update(dict(frame=i,time_s=t,world=surface['world'],query_pixel_id=surface['query_pixel_id'],partner_pixel_id=surface['partner_pixel_id'],input_sha256=hashes),t)
                (out/'source_rows.json').write_text(json.dumps(source_rows,indent=2)+'\n')
                np.savez_compressed(out/'frames'/f'frame_{i:03d}.npz',**rgb,**transforms,
                    left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),
                    left_camera_usd_world=get_cam_pose(stage,LEFT_CAMERA_PATH),right_camera_usd_world=get_cam_pose(stage,RIGHT_CAMERA_PATH),
                    root_position_world_m=position,root_quaternion_world_wxyz=q,time_s=step.simulation_time_s)
            self._current_policy_step=step
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(IsaacRuntimeConfig(scene_root=Path(config['scene_root']),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,
        record_video=True,occupancy_path=Path(config['occupancy']),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),
        lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',diagnostic_open_plane=False,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3))
    code=1;journal=None;shadow=None
    try:
        runtime.reset(episode)
        termination='timeout35'
        protocol=load_action_protocol(DEFAULT_CONTRACT.resolve())
        policy=with_heading_rate_policy(with_command_domain_cap(with_cusp_coast_acquisition(with_first_primitive_direction(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(PriorRGBPolicy))))),enabled=a.arm=='candidate')(protocol,memory_mode='current')
        policy.reset(EpisodeContext(episode.episode_id,False,None,protocol.protocol_id))
        policy.grid=RepeatedExposureMemoryGrid(np.asarray(episode.start_position_world_m[:2])-20,10,static_scene_assumption=True)
        observation_adapter=EstimatedNavigationObservation();journal=PolicyRowJournal(out/'policy_rows.ndjson')
        baseline=with_front_view_baseline_shadow(with_command_domain_cap(with_cusp_coast_acquisition(with_first_primitive_direction(with_tube_viability_after_measured_reversal(with_supported_four_points(PriorRGBPolicy))))))(protocol,memory_mode='current')
        baseline.reset(EpisodeContext(episode.episode_id,False,None,protocol.protocol_id));baseline.grid=RepeatedExposureMemoryGrid(np.asarray(episode.start_position_world_m[:2])-20,10,static_scene_assumption=True)
        shadow=ReadOnlyBoundaryShadow(baseline);prefix_gate=None;acquisition=FrontViewAcquisition()

        initial=export_execution(runtime._action_smoother,runtime._action_protocol)
        assert initial['pid_config']==config['pid_config'] and initial['inner_config']==config['inner_config']
        (out/'initial_execution_after_reset.json').write_text(json.dumps(initial,indent=2)+'\n')
        for block in range(350):
            available=runtime._current_policy_step
            if available.collision:termination='collision';break
            if np.linalg.norm(np.asarray(available.observation.position_world_xy)-np.asarray(episode.goal_position_world_m[:2]))<=.2:termination='success';break
            action_wall_begin=time.monotonic()
            t=float(clocks[-1]['episode_time_s']);source=source_rows[-1];state=states[-1]
            R=Rotation.from_quat(np.asarray(state['quaternion'])[[1,2,3,0]]).as_matrix()
            pose=dict(time_s=t,root_position_world_m=np.asarray(state['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(state['velocity']))
            gyro=R.T@np.asarray(runtime._robot.get_angular_velocity())
            observation,local_goal=observation_adapter.build(runtime._current_policy_step.observation,pose,gyro,episode.goal_position_world_m)
            snapshot=raw_point_memory.at(t);risk_source=dict(frame=source['frame'],time_s=source['time_s'],records=snapshot['records'])
            packet=dict(frame=block,time_s=t,points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m']);policy.set_source_packet(risk_source)
            policy_begin=time.monotonic();request=acquisition.choose(t,pose['root_position_world_m'][:2],float(np.linalg.norm(observation.proprio_history[-1,:2])),risk_source)
            if acquisition.failed:termination='REJECT_INITIAL_VIEW_HANDOFF_TIMEOUT';break
            prefix_active=request is not None
            if prefix_active:
                policy.grid.set_source(risk_source);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t)
                observe_clock_without_act(policy,packet,t)
                policy.records.append(dict(frame=block,time_s=t,request=request.tolist(),source_layer='RGB_partial_surface_initial_view_acquisition',source_frame=source['frame'],source_time_s=source['time_s'],source_age_s=t-source['time_s'],tracker_fields_valid=False,unknown_fraction=1.,local_reference_version=0,**policy.grid.diagnostics()))
            else:
                policy.set_current_packet(packet);request=policy.act(observation,local_goal)
            policy.records[-1].update(front_view_acquisition=acquisition.diagnostics,prefix_observer=prefix_active,planner_act_called=not prefix_active)
            policy_wall=time.monotonic()-policy_begin
            shadow_row=shadow.observe(request,observation,local_goal,dict(frame=source['frame'],time_s=source['time_s'],records=baseline_point_memory.at(t)['records']),dict(frame=block,time_s=t,points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m']))
            if shadow_row is not None:
                shadow_row['B0_state_after_act']=boundary_state(baseline.tracker)
                with (out/'B0_shadow_rows.ndjson').open('a') as shadow_log:shadow_log.write(json.dumps(shadow_row,separators=(',',':'))+'\n')
            row=dict(policy.records[-1]);row.update(policy_wall_s=policy_wall,frame=block,time_s=t,known_GT_pose=True,native_pose_used=False,prefix_observer=prefix_active,planner_act_called=not prefix_active,tracker_state_after_act=boundary_state(policy.tracker),installed_reference_xy_after_act=np.asarray(policy.tracker._reference_xy).tolist(),raw_point_memory_current=snapshot['current_count'],raw_point_memory_past=snapshot['historical_count'],partial_source_not_full_volume=True,lowlevel_PID_ideal_IMU_privileged=True)
            if policy.plan is not None:
                (out/'local_plans'/f'frame_{block:03d}.json').write_text(json.dumps(dict(frame=block,time_s=t,reference_version=policy.local_reference_version,status=policy.plan.status,waypoints=[dataclasses.asdict(w) for w in policy.plan.waypoints],expanded_nodes=policy.plan.expanded_nodes,path_length_m=policy.plan.path_length_m,direction_changes=policy.plan.direction_changes,plan_meta=policy.plan_meta))+'\n')
            serial={key:value for key,value in snapshot.items() if key!='records'}
            serial['records']=[{key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in record.items()} for record in snapshot['records']]
            (out/'raw_point_memory_snapshots'/f'frame_{block:03d}.json').write_text(json.dumps(serial)+'\n')
            grid=policy.grid;np.savez_compressed(out/'maps'/f'frame_{block:03d}.npz',labels=grid.labels,codes=grid.codes,hazard_acquisition_time=grid.hazard_acquisition_time,origin=grid.origin,resolution=grid.resolution,ground_prior=grid.task_ground_prior,safe_start_prior=grid.safe_start_prior,time_s=t)
            journal.planned(row)
            actual_requests.append(request.tolist())
            execute_wall_begin=time.monotonic();current=runtime.execute(request)
            row.update(execute_capture_NN_wall_s=time.monotonic()-execute_wall_begin,end_to_end_wall_s=time.monotonic()-action_wall_begin,wall_includes_policy_execute_render_NN_and_capture=True)
            journal.executed(row)
            # Actual contact stops the run; measured XY goal entry is conditional TaskH acceptance only.
            if current.collision:termination='collision';break
            goal_remaining=float(np.linalg.norm(np.asarray(current.observation.position_world_xy)-np.asarray(episode.goal_position_world_m[:2])))
            if goal_remaining<=.2:termination='success';break
        (out/'termination.json').write_text(json.dumps(dict(termination=termination,actions=len(actual_requests),time_s=clocks[-1]['episode_time_s'],budget_s=35.,true_goal_xy_threshold_m=.2,not_original35=False,original35_time_budget=True,original_strict_cold_start_task=False,not_causal_navigation_AB=True))+'\n')
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():
            write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        (out/'states.json').write_text(json.dumps(states,indent=2)+'\n')
        (out/'camera_clocks.json').write_text(json.dumps(clocks,indent=2)+'\n')
        (out/'physics_contacts.json').write_text(json.dumps(runtime.collision_contact_samples,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(name=a.name,role='One projected heading-rate feedback consumer; same sparse matcher handoff planner tracker PID speed; knownpose TaskH development',live_packet_sampler='original_stride2',within_arm_shadow_not_cross_arm_counterfactual=True,actual_source_class=str(type(depth_model)),frozen_source_method='SGBMStereo_448_block5_LR1p5',complete=True,front_view_acquisition=acquisition.diagnostics,actions=len(actual_requests),captures=len(clocks),saved_frames=len(source_rows),reset_count=1,internal_reset=False,recorded_video_frames=len(clocks),acquisition='fixed3 -> singlebasecapture -> fixed3 -> imagesonly',camera_rig=camera_rig_metadata(),pid_config=initial['pid_config'],inner_config=initial['inner_config'],runtime_class=str(type(runtime)),policy_MRO=[str(c) for c in type(policy).__mro__],tracker_MRO=[str(c) for c in type(policy.tracker).__mro__],baseline_policy_MRO=[str(c) for c in type(baseline).__mro__],baseline_tracker_MRO=[str(c) for c in type(baseline.tracker).__mro__],heuristic_not_safe_stopping_bound=True,full_initial_state_coverage_not_proven=True,known_GT_pose_control=True,source_GT_pose_registration=True,frozen_settling_s=6.,no_prescribed_reverse_prefix=True,pid_module_file=str(sys.modules['roll_pid'].__file__),source_sha256={file:hashlib.sha256(Path(file).read_bytes()).hexdigest() for file in config['source_sha256']},input_sha256=config['input_sha256'],no_model_fit=True,no_controller_tuning=True,heading_rate_feedback_enabled=a.arm=='candidate',only_control_change='measured_rate_in_existing_outer_yaw_correction',extra_physics_tail=False),indent=2)+'\n')
        code=0;print('CONDITIONAL_TASK_RUN_COMPLETE',a.name,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        def partial_export():
            np.savez(out/'requests.npz',requests=np.asarray(actual_requests,np.float32).reshape(-1,10))
            for name,data in [('native_imu_reads',imu_reads),('native_imu_packets',imu_packets),('native_predictions',imu_predictions),('initial_gauge',gauge),('camera_clocks',clocks),('states',states),('physics_contacts',runtime.collision_contact_samples),('native_imu_identity',dict(creation=imu_creation,initial_normal=None if imu_initial_normal is None else imu_initial_normal.tolist(),orientation_field_read=False,predictions_used_by_control=False,known_GT_pose_control=True,no_prescribed_observation_prefix=True))]:
                (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
            np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
            if code!=0:
                for name,frames in runtime.captured_frames.items():
                    if frames:write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
            if shadow is not None:(out/'B0_shadow_until_fork.json').write_text(json.dumps(shadow.rows,indent=2)+'\n')
            journal_path=out/'policy_rows.ndjson'
            if journal_path.exists():
                if journal is not None:journal.close()
                recover_to_files(journal_path,out/'policy_records.json',out/'journal_recovery.json')
        export_then_close(runtime,partial_export,code)
if __name__=='__main__':main()
