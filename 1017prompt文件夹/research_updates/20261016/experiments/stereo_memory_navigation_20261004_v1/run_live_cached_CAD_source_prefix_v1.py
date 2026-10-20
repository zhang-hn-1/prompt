"""ONE60s closed-loop current-native-camera observation pilot, independent partial-point navigation."""
import argparse,hashlib,json,sys,traceback,dataclasses,time
from pathlib import Path
import numpy as np
import cv2
from fast_stereo import FastStereo
from masked_source_pair import paired_sources
from timed_static_base_self_mask import TimedStaticBaseSelfMask as RobotSelfMask
from stereo_geometry import restore_native
from paired_texture_surface import PairedTextureSurfaceMemory
from partial_point_risk_grid import PartialPointRiskGrid
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from estimated_navigation_observation import EstimatedNavigationObservation
from observation_clock_adapter import observe_clock_without_act
from guarded_reference import boundary_state
from benchmark.policies import EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from tentative_point_memory import TentativePointMemory
from fixed_appearance_intervention_v1 import bind_fixed_appearance
from live_refined_surface_shadow import refined_surface
from reverse_bank_observation import ReverseBankObservation,current_observer_inputs
from imu_gravity_rolling import GravityRolling
from imu_interval_packet import interval_packet
from scipy.spatial.transform import Rotation
from execution_snapshot import export_execution
from probe_cleanup import install_sigterm_cleanup,export_then_close
from native_probe_identity import export_task_identity
from scripts.camera_rig import camera_rig_metadata
from render_flush import fixed_render_flush
from capture_transaction import replace_step_images
from benchmark.artifacts import write_rgb_video
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from scripts.run_round4_fixed_reference import load_transformed_reference
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
    a=argparse.Namespace(name='live_cached_CAD_source_prefix')
    config=json.loads((H/'live_cached_CAD_source_prefix_protocol_v1.json').read_text())
    for file,digest in config['source_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    for file,digest in config['input_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    out=H/'live_cached_CAD_source_prefix_run_v1';out.mkdir(parents=True,exist_ok=False)
    (out/'frames').mkdir()
    for n in ['source_predictions','sensor_packets','estimated_poses','partial_observations','source_inputs','partial_class_masks','refined_shadow_points','tentative_memory_snapshots','raw_point_memory_snapshots','maps','local_plans']:(out/n).mkdir()
    source_model=np.load(config['appearance_model']);extrinsics=np.load(config['rig_extrinsics'])
    static_masks=np.load(config['static_masks']);surface_memory=PairedTextureSurfaceMemory();tentative_memory=TentativePointMemory();raw_point_memory=TentativePointMemory();shadow_rows=[];depth_model=None;self_masker=None;source_rows=[]
    commands=np.asarray(json.loads(Path(config['command_plan']).read_text()),dtype=np.float32);assert commands.shape==(140,10);np.savez(out/'planned_parent_requests.npz',requests=commands)
    observer=ReverseBankObservation(np.array([-2.2,.15]));observer_inputs=[];actual_requests=[]
    ref,_=load_transformed_reference(H.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
    episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
    episode=dataclasses.replace(episode,episode_id='live_cached_CAD_source_prefix',scene_uri='${SCENE_ROOT}/single_cylinder.usda',scene_sha256=hashlib.sha256((H/'fixed_appearance_fixture_v1/single_cylinder.usda').read_bytes()).hexdigest(),occupancy_sha256=hashlib.sha256((H/'fixed_appearance_fixture_v1/occupancy.npz').read_bytes()).hexdigest(),start_quaternion_world_wxyz=tuple(config['start_quaternion_world_wxyz']),start_position_world_m=(-2.2,.15,.45),goal_position_world_m=(2.2,.15,.45),euclidean_distance_m=4.4)
    export_task_identity(out/'task_episode.json',episode)
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
            binding=bind_fixed_appearance(stage,H/'fixed_appearance_fixture_v1/fixed_noise_seed20261008.png')
            (out/'appearance_binding.json').write_text(json.dumps(binding,indent=2)+'\n')
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
                # Frozen native pose + frozen rig only; no current GT delivered to source.
                native=imu_predictions[-1];assert native['available'] and abs(native['time_s']-float(step.simulation_time_s))<1e-8
                rootT=np.eye(4);rootT[:3,:3]=native['rotation_world'];rootT[:3,3]=native['position_world']
                source_poses={}
                for eye in ('left','right'):
                    assert np.allclose(transforms[eye+'_optical_to_base'],extrinsics[eye+'_optical_to_base'],atol=1e-6)
                    optical=rootT@extrinsics[eye+'_optical_to_base'];usd=optical.copy();usd[:3,:3]=optical[:3,:3]@np.diag([1,-1,-1]);source_poses[eye+'_camera_usd_world']=usd
                t=float(step.simulation_time_s)
                np.savez_compressed(out/'source_inputs'/f'frame_{i:03d}.npz',**rgb,time_s=t)
                np.savez_compressed(out/'estimated_poses'/f'frame_{i:03d}.npz',**source_poses,root_position_world_m=rootT[:3,3],time_s=t)
                if depth_model is None:depth_model=FastStereo()
                pred=depth_model.predict(rgb['left_rgb'],rgb['right_rgb'],static_masks['left'],static_masks['right'])
                np.savez_compressed(out/'source_predictions'/f'frame_{i:03d}.npz',**pred,frame=i,time_s=t)
                if self_masker is None:self_masker=RobotSelfMask()
                proprio=step.observation.proprio_history.copy();proprio[:,:9]=0.
                _,source_packet,current_masks=paired_sources(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(**source_poses,root_position_world_m=rootT[:3,3],time_s=t),proprio,self_masker,t)
                np.savez_compressed(out/'sensor_packets'/f'source_{i:03d}.npz',**source_packet)
                classes={}
                for eye in ('left','right'):
                    lab=cv2.cvtColor(rgb[eye+'_rgb'],cv2.COLOR_RGB2LAB).astype(float)
                    classes[eye]=(np.linalg.norm((lab-source_model['centre'])/source_model['scale'],axis=-1)<=3)&~current_masks[eye]
                np.savez_compressed(out/'partial_class_masks'/f'frame_{i:03d}.npz',**classes,frame=i,time_s=t,appearance_frozen_before_sequence=True)
                sourcefiles=[out/n/f'frame_{i:03d}.npz' for n in ('source_inputs','estimated_poses','source_predictions','partial_class_masks')]+[out/'sensor_packets'/f'source_{i:03d}.npz']
                hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sourcefiles}
                surface=surface_memory.update(source_packet,rgb['left_rgb'],rgb['right_rgb'],t,hashes,restore_native(pred['disparity_px']))
                np.savez_compressed(out/'partial_observations'/f'frame_{i:03d}.npz',**{k:v for k,v in surface.items() if k!='input_hashes'})
                source_rows.append(dict(frame=i,time_s=t,points=len(surface['world']),CAD_projection_wall_s=self_masker.last_project_wall_s,provenance=surface['input_hashes'],memory_acquisition_times_s=[v['acquisition_time_s'] for v in surface_memory.at(t)],truth_read_after_output=True))
                (out/'source_rows.json').write_text(json.dumps(source_rows,indent=2)+'\n')
                raw_record=dict(frame=i,time_s=t,world=surface['world'],query_pixel_id=surface['query_pixel_id'],partner_pixel_id=surface['partner_pixel_id'],input_sha256=hashes)
                raw_point_memory.update(raw_record,t)
                raw_snapshot=raw_point_memory.at(t)
                raw_serial={k:v for k,v in raw_snapshot.items() if k!='records'}
                raw_serial['records']=[{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in r.items()} for r in raw_snapshot['records']]
                (out/'raw_point_memory_snapshots'/f'frame_{i:03d}.json').write_text(json.dumps(raw_serial)+'\n')
                # Read-only diagnostic interface: parent observer consumes original source_rows.
                before_shadow=snapshot();shadow_clock=self._clock_metadata()
                execution_before=export_execution(self._action_smoother,self._action_protocol) if hasattr(self._action_smoother,'pid') else None
                begin_shadow=time.monotonic()
                refined=refined_surface(surface,rgb['left_rgb'],rgb['right_rgb'],current_masks,classes['right'],restore_native(pred['disparity_px']),source_poses['left_camera_usd_world'],t,hashes)
                tentative_memory.update(refined,t);memory=tentative_memory.at(t)
                np.savez_compressed(out/'refined_shadow_points'/f'frame_{i:03d}.npz',**{k:v for k,v in refined.items() if k!='input_sha256'})
                serial={k:v for k,v in memory.items() if k!='records'}
                serial['records']=[{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in r.items()} for r in memory['records']]
                (out/'tentative_memory_snapshots'/f'frame_{i:03d}.json').write_text(json.dumps(serial,indent=2)+'\n')
                execution_after=export_execution(self._action_smoother,self._action_protocol) if hasattr(self._action_smoother,'pid') else None
                assert shadow_clock==self._clock_metadata() and all(np.array_equal(a,b) for a,b in zip(before_shadow,snapshot())) and execution_before==execution_after
                shadow_rows.append(dict(frame=i,time_s=t,current_count=memory['current_count'],historical_count=memory['historical_count'],distinct_XY_cells=memory['distinct_XY_cells'],source_admitted=False,no_action_or_parent_count_change=True,physics_clock_and_execution_unchanged=True,wall_s=time.monotonic()-begin_shadow,input_sha256=hashes))
                (out/'memory_shadow_rows.json').write_text(json.dumps(shadow_rows,indent=2)+'\n')
                np.savez_compressed(out/'frames'/f'frame_{i:03d}.npz',**rgb,**transforms,
                    left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),
                    left_camera_usd_world=get_cam_pose(stage,LEFT_CAMERA_PATH),right_camera_usd_world=get_cam_pose(stage,RIGHT_CAMERA_PATH),
                    root_position_world_m=position,root_quaternion_world_wxyz=q,time_s=step.simulation_time_s)
            self._current_policy_step=step
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(IsaacRuntimeConfig(scene_root=H/'fixed_appearance_fixture_v1',contract_path=DEFAULT_CONTRACT.resolve(),gui=False,
        record_video=True,occupancy_path=H/'fixed_appearance_fixture_v1/occupancy.npz',drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),
        lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',diagnostic_open_plane=False,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3))
    runtime.set_reference_visualization_path(ref.points_xy)
    code=1
    try:
        runtime.reset(episode)
        protocol=load_action_protocol(DEFAULT_CONTRACT.resolve())
        policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current')
        policy.reset(EpisodeContext(episode.episode_id,False,None,protocol.protocol_id))
        navigation_observation=EstimatedNavigationObservation();policy_rows=[];native_low_count=0;initial_xy=np.asarray(imu_predictions[0]['position_world'])[:2];termination='source_interface_budget_complete'
        policy.grid=PartialPointRiskGrid(initial_xy-20,10)
        initial=export_execution(runtime._action_smoother,runtime._action_protocol)
        assert initial['pid_config']==config['pid_config'] and initial['inner_config']==config['inner_config']
        (out/'initial_execution_after_reset.json').write_text(json.dumps(initial,indent=2)+'\n')
        for block in range(140):
            action_wall_begin=time.monotonic()
            n=imu_predictions[-1];source=source_rows[-1];current_time=float(clocks[-1]['episode_time_s'])
            assert n['available'] and abs(n['time_s']-current_time)<1e-8
            pose=dict(time_s=current_time,root_position_world_m=np.asarray(n['position_world']),root_rotation_world=np.asarray(n['rotation_world']),velocity_world_m_s=np.asarray(n['velocity_world_m_s']))
            observation,local_goal=navigation_observation.build(runtime._current_policy_step.observation,pose,imu_packets[-1]['gyro_body_rad_s'],episode.goal_position_world_m)
            speed=float(np.linalg.norm(pose['velocity_world_m_s'][:2]));native_low_count=native_low_count+1 if speed<.05 else 0
            snapshot=raw_point_memory.at(current_time)
            risk_source=dict(frame=source['frame'],time_s=source['time_s'],records=snapshot['records'])
            current_packet=dict(frame=block,time_s=current_time,points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m'])
            if block<140:
                policy.grid.set_source(risk_source);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],current_time);observe_clock_without_act(policy,current_packet,current_time)
                parent_pair=commands[block].reshape(5,2)[0].copy()
                observer_input=current_observer_inputs(current_time,n,source,extrinsics['left_optical_to_base'],episode.goal_position_world_m[:2],parent_pair)
                v=observer_input;target=observer.request(v['time_s'],v['source_frame'],v['source_time_s'],v['point_count'],np.asarray(v['axis_world']),v['axis_time'],np.asarray(v['root_xy']),np.asarray(v['goal_xy']),np.asarray(v['parent']))
                request=np.tile(target,(5,1)).astype(np.float32).reshape(10);observer_inputs.append(observer_input);row=dict(frame=block,time_s=current_time,prefix_observer=True,planner_act_called=False,request=request.tolist())
            else:
                if block==140:
                    if native_low_count<5 or np.linalg.norm(pose['root_position_world_m'][:2]-initial_xy)>.7 or sum(v['points']>=100 for v in source_rows)<2:raise ValueError('first14 handoff failed; no wait/reset/retry')
                policy.set_source_packet(risk_source);policy.set_current_packet(current_packet);begin=time.monotonic();request=policy.act(observation,local_goal);wall=time.monotonic()-begin;row=dict(policy.records[-1]);row.update(prefix_observer=False,planner_act_called=True,policy_wall_s=wall,tracker_state_after_act=boundary_state(policy.tracker),installed_reference_xy_after_act=np.asarray(policy.tracker._reference_xy).tolist(),raw_point_memory_current=snapshot['current_count'],raw_point_memory_past=snapshot['historical_count'],partial_source_not_full_volume=True,native_pose=True,initial_gauge_privileged=True,lowlevel_PID_ideal_IMU_privileged=True)
            if block>=140 and policy.plan is not None:
                (out/'local_plans'/f'frame_{block:03d}.json').write_text(json.dumps(dict(frame=block,time_s=current_time,reference_version=policy.local_reference_version,status=policy.plan.status,waypoints=[dataclasses.asdict(w) for w in policy.plan.waypoints],expanded_nodes=policy.plan.expanded_nodes,path_length_m=policy.plan.path_length_m,direction_changes=policy.plan.direction_changes,plan_meta=policy.plan_meta))+'\n')
            grid=policy.grid;np.savez_compressed(out/'maps'/f'frame_{block:03d}.npz',labels=grid.labels,codes=grid.codes,hazard_acquisition_time=grid.hazard_acquisition_time,origin=grid.origin,resolution=grid.resolution,ground_prior=grid.task_ground_prior,safe_start_prior=grid.safe_start_prior,time_s=current_time)
            policy_rows.append(row);(out/'policy_records.json').write_text(json.dumps(policy_rows,indent=2)+'\n');actual_requests.append(np.asarray(request).tolist())
            execute_wall_begin=time.monotonic();current=runtime.execute(request)
            row.update(execute_capture_NN_wall_s=time.monotonic()-execute_wall_begin,end_to_end_wall_s=time.monotonic()-action_wall_begin,wall_includes_policy_execute_render_NN_and_capture=True)
            (out/'policy_records.json').write_text(json.dumps(policy_rows,indent=2)+'\n')
            # Evaluation-only terminal truth is never supplied to source/policy.
            if current.collision:termination='collision';break
            if np.linalg.norm(np.asarray(current.observation.position_world_xy)-np.asarray(episode.goal_position_world_m[:2]))<=.2:termination='success';break
        (out/'termination.json').write_text(json.dumps(dict(termination=termination,actions=len(actual_requests),time_s=clocks[-1]['episode_time_s'],true_goal_xy_threshold_m=.2,budget_s=14,not_original35=True,not_causal_navigation_AB=True))+'\n')
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():
            write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        (out/'states.json').write_text(json.dumps(states,indent=2)+'\n')
        (out/'camera_clocks.json').write_text(json.dumps(clocks,indent=2)+'\n')
        (out/'physics_contacts.json').write_text(json.dumps(runtime.collision_contact_samples,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(name=a.name,role='ONE14s cached-CAD online source interface; no navigation act/no35',complete=True,actions=len(actual_requests),captures=len(clocks),saved_frames=len(source_rows),reset_count=1,internal_reset=False,recorded_video_frames=len(clocks),acquisition='fixed3 -> singlebasecapture -> fixed3 -> imagesonly',camera_rig=camera_rig_metadata(),pid_config=initial['pid_config'],inner_config=initial['inner_config'],runtime_class=str(type(runtime)),pid_module_file=str(sys.modules['roll_pid'].__file__),source_sha256={file:hashlib.sha256(Path(file).read_bytes()).hexdigest() for file in config['source_sha256']},input_sha256=config['input_sha256'],no_model_fit=True,no_controller_tuning=True,extra_physics_tail=False),indent=2)+'\n')
        code=0;print('CONDITIONAL_TASK_RUN_COMPLETE',a.name,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        def partial_export():
            np.savez(out/'requests.npz',requests=np.asarray(actual_requests,np.float32).reshape(-1,10))
            (out/'observer_inputs.json').write_text(json.dumps(observer_inputs,indent=2)+'\n')
            (out/'observer_records.json').write_text(json.dumps(observer.records,indent=2)+'\n')
            for name,data in [('native_imu_reads',imu_reads),('native_imu_packets',imu_packets),('native_predictions',imu_predictions),('initial_gauge',gauge),('camera_clocks',clocks),('states',states),('physics_contacts',runtime.collision_contact_samples),('native_imu_identity',dict(creation=imu_creation,initial_normal=None if imu_initial_normal is None else imu_initial_normal.tolist(),orientation_field_read=False,predictions_used_by_control=True))]:
                (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
            np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
            if code!=0:
                for name,frames in runtime.captured_frames.items():
                    if frames:write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        export_then_close(runtime,partial_export,code)
if __name__=='__main__':main()
