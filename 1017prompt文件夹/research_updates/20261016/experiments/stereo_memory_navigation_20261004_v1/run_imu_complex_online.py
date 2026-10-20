"""ONE original35s current-rig Oracle episode with genuinely live RGB-only shadow."""
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
from rolling_shadow import OnlineRollingShadow,emulate_sensor_row
from shell_link_measurement import copy_link_state
from imu_gravity_rolling import GravityRolling
from imu_interval_packet import interval_packet
from complete_capture import complete_capture
from render_flush import fixed_render_flush
from oracle_rgb_shadow import OnlineRGBShadow,make_oracle,reference_from_arrays
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
    config=json.loads((H/'imu_complex_online_protocol.json').read_text())
    for f,s in config['source_sha256'].items():assert sha(f)==s,f
    out=H/'imu_complex_online_run';out.mkdir(exist_ok=False)
    for name in ['online_inputs','known_poses','offline_truth','history','shadow','policy_inputs']:(out/name).mkdir()
    episode=next(e for e in runner.load_manifest(config['manifest']).episodes if e.episode_id==config['episode_id'])
    occupancy=runner._resolve_occupancy_path(P/'configs/benchmark_v1/scene_index.json',episode,P/'data')
    validate_mask_identity(json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text()))
    maskdata=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');shadow=OnlineRGBShadow({eye:maskdata[eye] for eye in ['left','right']})
    rolling=OnlineRollingShadow()
    rolling_live_identity=None
    actual_links=[];actual_link_identity=None
    imu_reads=[];imu_predictions=[];imu_packets=[];imu_model=None;imu_initial_normal=None;imu_interface=None;imu_path=None;imu_creation=None
    clocks=[];states=[];execution_states=[];contacts=[];markers=[];policies=[]
    class CaptureRuntime(runner.IsaacBenchmarkRuntime):
        def _configured_articulation_gains(self):
            gains=super()._configured_articulation_gains()
            if not hasattr(self,'_interval_original_step'):
                self._interval_original_step=self._world.step
                def measured_step(*args,**kwargs):
                    begin=float(self._world.current_time)
                    result=self._interval_original_step(*args,**kwargs)
                    self._actual_last_interval=(begin,float(self._world.current_time))
                    return result
                self._world.step=measured_step
            return gains
        def _record_physics_diagnostic(self,*args,**kwargs):
            nonlocal rolling_live_identity,actual_link_identity,imu_model,imu_initial_normal
            super()._record_physics_diagnostic(*args,**kwargs)
            if rolling_live_identity is None:
                names=list(self._robot.dof_names)
                assert names==['joint1','joint2','joint3'],names
                rolling_live_identity=dict(dof_names=names,main_encoder_index=0,radius_m=.4000000059604645,axis_body=[0,1,0],sensor_role='ideal simulated body gyro; zero noise/bias; per-step GT used only inside emulation, no GT position delivered')
            sensor,normal=emulate_sensor_row(self._physics_diagnostics)
            assert rolling.consume(sensor,normal) is None
            # Read existing physical handle only; no setters, state refresh, physics or policy calls.
            articulation=self._robot._articulation_view
            assert articulation.is_physics_handle_valid()
            view=articulation._physics_view
            names=list(view.shared_metatype.link_names)
            pose=np.asarray(view.get_link_transforms()).copy()
            velocity=np.asarray(view.get_link_velocities()).copy()
            linkrow=copy_link_state(sensor['time_s'],names,pose,velocity)
            if actual_link_identity is None:
                actual_link_identity=dict(link_names=names,base_index=names.index('base_link'),shell_index=names.index('link1'),dof_names=list(self._robot.dof_names),com_local_xyzw=np.asarray(view.get_coms()).copy().tolist(),view_class=str(type(view)),root_time_same=True,read_only=True,pose_order='position xyz / quaternion xyzw (root comparison required)',velocity_order='linear xyz / angular xyz (COM convention separately audited)')
            actual_links.append(linkrow)
            # Native SDK sample is read NOW; orientation is never accessed.
            if imu_initial_normal is None:imu_initial_normal=np.asarray(normal).copy()
            reading=imu_interface.get_sensor_reading(imu_path,use_latest_data=True,read_gravity=True)
            begin,end=self._actual_last_interval
            assert abs(end-self._physics_diagnostics['raw_simulation_time_s'][-1])<=1e-10
            packet,diagnostic=interval_packet(reading,begin,end,sensor['time_s'],sensor['main_encoder_rad_s'])
            imu_reads.append(diagnostic)
            if packet is None:
                imu_predictions.append(dict(time_s=sensor['time_s'],available=False,reason=diagnostic['reason']))
                imu_packets.append(None)
            else:
                if imu_model is None:imu_model=GravityRolling(imu_initial_normal)
                prediction=imu_model.update(packet)
                imu_packets.append({k:np.asarray(v).tolist() for k,v in packet.items()})
                imu_predictions.append(dict(time_s=sensor['time_s'],available=True,**{k:np.asarray(v).tolist() for k,v in prediction.items()},live_before_next_action=True))
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            result=runner.IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom)
            # Actual lifecycle: after first warmup, BEFORE existing formal articulation world.reset.
            nonlocal imu_interface,imu_path,imu_creation
            import omni.kit.commands
            from isaacsim.sensors.physics import _sensor
            from collect_data_dubins import BASE_ROOT
            imu_path=BASE_ROOT+'/research_native_imu'
            ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/research_native_imu',parent=BASE_ROOT,sensor_period=.02,translation=gf.Vec3d(0,0,0),orientation=gf.Quatd(1,0,0,0),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1)
            assert ok and prim is not None
            imu_interface=_sensor.acquire_imu_sensor_interface()
            imu_creation=dict(path=imu_path,period_s=.02,filter_sizes=[1,1,1],translation=[0,0,0],orientation_local_wxyz=[1,0,0,0],read_gravity=True,use_latest_data=True,lifecycle='after warmup/before original formal articulation world.reset',extra_reset=False,no_physics_or_colliders_added=True)
            return result
        def _capture(self):
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
            shadow_start=time.perf_counter()
            packet=dict(frame=i,time_s=t,**rgb,**poses)
            prediction=shadow.observe(packet)
            live=shadow.latest
            fp=out/'shadow'/f'features_{i:03d}.npz';np.savez_compressed(fp,**live['features'],frame=i,time_s=t)
            mp=out/'shadow'/f'map_{i:03d}.npz';s=live['map'];np.savez_compressed(mp,**{k:s[k] for k in ['labels','provenance','floor_time','hazard_time']},time_s=t)
            prediction.update(features_sha256=sha(fp),map_sha256=sha(mp),rgb_sha256=sha(out/'online_inputs'/f'frame_{i:03d}.npz'),poses_sha256=sha(out/'known_poses'/f'frame_{i:03d}.npz'),capture_wall_s=time.perf_counter()-capture_started,shadow_serialized_before_next_action=True)
            (out/'shadow'/f'prediction_{i:03d}.json').write_text(json.dumps(prediction,indent=2)+'\n')
            clocks[-1]['shadow_wall_s']=time.perf_counter()-shadow_start
            np.savez_compressed(out/'offline_truth'/f'frame_{i:03d}.npz',**poses,**transforms,left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),root_position_world_m=p,root_quaternion_world_wxyz=q,time_s=t)
            if i%30==0:print('CAPTURE',i,t,flush=True)
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=Path('/home/an/project/NavDP/assets/scenes'),contract_path=P/'configs/data_contract.yaml',occupancy_path=occupancy,record_video=True,gui=False,diagnostic_physics_trace=True,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',auxiliary_velocity_limit_rad_s=.3))
    protocol=runner.load_action_protocol(P/'configs/data_contract.yaml');policy=make_oracle(protocol)
    ref=np.load(H/'oracle_rgb_shadow_reference.npz');plan=reference_from_arrays(ref['pose'],ref['direction'],ref['command'],config['reference_length_m'],config['reference_expanded_nodes'])
    act=policy.act
    def recorded_act(observation,goal):
        i=len(policies);action=act(observation,goal)
        np.savez_compressed(out/'policy_inputs'/f'frame_{i:03d}.npz',left_rgb=observation.stereo_left_rgb,right_rgb=observation.stereo_right_rgb,proprio_history=observation.proprio_history,action_history=observation.action_history,quaternion_history=observation.quaternion_world_wxyz_history,position_world_xy=observation.position_world_xy,yaw_world_rad=observation.yaw_world_rad,pointgoal_base_xy=goal,request=action)
        policies.append(dict(frame=i,diagnostics=policy.diagnostics,request=policy.request_diagnostics,tracking=policy.tracking_diagnostics,mapping=policy.mapping_records[-1] if policy.mapping_records else None))
        return action
    policy.act=recorded_act
    result=None;code=1
    try:
        result=run_episode(runtime,policy,episode,action_protocol=protocol,config=RunnerConfig(),kinodynamic_plan=plan)
        arrays={f.name:getattr(result,f.name) for f in dataclasses.fields(result) if isinstance(getattr(result,f.name),np.ndarray)}
        np.savez_compressed(out/'episode_arrays.npz',**arrays)
        (out/'metrics.json').write_text(json.dumps(result.to_metrics_dict(),indent=2)+'\n')
        np.savez_compressed(out/'oracle_path.npz',xy=runtime.oracle_path_xy(episode))
        occ=runtime.occupancy;np.savez_compressed(out/'offline_occupancy_label.npz',labels=occ.grid,origin=occ.global_origin_xy,resolution=occ.resolution_m)
        code=0 if result.runtime_error is None else 1
        print('ORACLE_RGB_SHADOW_COMPLETE',result.termination_reason,result.elapsed_s,result.final_distance_m,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        (out/'native_imu_reads.json').write_text(json.dumps(imu_reads,indent=2)+'\n')
        (out/'native_imu_predictions.json').write_text(json.dumps(imu_predictions,indent=2)+'\n')
        (out/'native_imu_packets.json').write_text(json.dumps(imu_packets,indent=2)+'\n')
        (out/'native_imu_identity.json').write_text(json.dumps(dict(creation=imu_creation,initial_normal=None if imu_initial_normal is None else imu_initial_normal.tolist(),orientation_field_read=False,predictions_used_by_control=False),indent=2)+'\n')
        (out/'actual_link_states.json').write_text(json.dumps(actual_links,indent=2)+'\n')
        (out/'actual_link_identity.json').write_text(json.dumps(actual_link_identity,indent=2)+'\n')
        (out/'online_rolling_predictions.json').write_text(json.dumps(rolling.records,indent=2)+'\n')
        (out/'ideal_sensor_inputs.json').write_text(json.dumps(rolling.sensors,indent=2)+'\n')
        (out/'rolling_live_identity.json').write_text(json.dumps(dict(live=rolling_live_identity,initial_gravity_normal=None if rolling.initial_normal is None else rolling.initial_normal.tolist(),estimator_does_not_read_positions=True,estimator_does_not_read_dynamic_gt_quaternions=True,controller_does_not_consume_estimator=True),indent=2)+'\n')
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        for name,data in [('camera_clocks',clocks),('states',states),('execution_states',execution_states),('contacts',contacts),('marker_checks',markers),('physics_contacts',runtime.collision_contact_samples),('policy_records',policies)]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(completed=code==0,policy_calls=len(policies),captures=len(clocks),reset_count=1,internal_reset=False,scene_sha256=episode.scene_sha256,occupancy_sha256=episode.occupancy_sha256,camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',controller_class=str(type(policy)),runtime_class=str(type(runtime)),source_sha256={f:sha(f) for f in config['source_sha256']},oracle_only_reference=True,shadow_influences_control=False,source_model_admitted=False,extra_physics_tail=False),indent=2)+'\n')
        manifest={str(p):sha(p) for p in out.rglob('*') if p.is_file()}
        (out/'artifact_manifest.json').write_text(json.dumps(dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact_sha256=manifest,source_model_admitted=False,shadow_action_authorized=False),indent=2)+'\n')
        runtime.close(return_code=code)
if __name__=='__main__':main()
