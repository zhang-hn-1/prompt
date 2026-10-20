"""ONE original35s privileged depth/local curve package; no global reference input."""
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
from root_direction_consumer import RootDirectionPolicy
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
    config=json.loads((H/'root_direction_protocol.json').read_text())
    for f,s in config['source_sha256'].items():assert sha(f)==s,f
    out=H/'root_direction_run';out.mkdir(exist_ok=False)
    for name in ['online_inputs','known_poses','offline_truth','history','sensor_packets','maps','local_plans','policy_inputs']:(out/name).mkdir()
    episode=next(e for e in runner.load_manifest(config['manifest']).episodes if e.episode_id==config['episode_id'])
    occupancy=runner._resolve_occupancy_path(P/'configs/benchmark_v1/scene_index.json',episode,P/'data')
    validate_mask_identity(json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text()))
    maskdata=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    yy,xx=np.mgrid[0:480:4,0:640:4]
    rays=np.column_stack(((xx.ravel()-319.5)/584.8888888888889,(yy.ravel()-239.5)/584.8888888888889,np.ones(xx.size)))
    clocks=[];states=[];execution_states=[];contacts=[];markers=[];policies=[];current_packet=None
    class CaptureRuntime(runner.IsaacBenchmarkRuntime):
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            result=runner.IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom);return result
        def _capture(self):
            nonlocal current_packet
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
            # Explicit privileged sensor layer: both CURRENT measured depths and known camera poses.
            # No occupancy/reference/filter labels are read for constructing these points.
            point_started=time.perf_counter();points=[];depths={}
            for eye,cam in [('left',self._left_camera),('right',self._right_camera)]:
                d=read_camera_depth(cam);depths[eye+'_depth_m']=d
                values=d[::4,::4].ravel();pose=poses[eye+'_camera_usd_world']
                rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
                valid=np.isfinite(values)&(values>=.45)&(values<=8.)&(~maskdata[eye][::4,::4].ravel())
                points.append(pose[:3,3]+(rays[valid]*values[valid,None])@rotation.T)
            packet=dict(frame=i,time_s=t,points_world=np.concatenate(points),root_xyz=np.asarray(p))
            np.savez_compressed(out/'sensor_packets'/f'frame_{i:03d}.npz',**packet)
            current_packet=packet
            clocks[-1]['depth_point_wall_s']=time.perf_counter()-point_started
            clocks[-1]['capture_partial_wall_s']=time.perf_counter()-capture_started
            np.savez_compressed(out/'offline_truth'/f'frame_{i:03d}.npz',**poses,**transforms,**depths,root_position_world_m=p,root_quaternion_world_wxyz=q,time_s=t)
            if i%30==0:print('CAPTURE',i,t,flush=True)
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=Path('/home/an/project/NavDP/assets/scenes'),contract_path=P/'configs/data_contract.yaml',occupancy_path=occupancy,record_video=True,gui=False,diagnostic_physics_trace=True,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',auxiliary_velocity_limit_rad_s=.3))
    protocol=runner.load_action_protocol(P/'configs/data_contract.yaml');policy=RootDirectionPolicy(protocol)
    act=policy.act
    def recorded_act(observation,goal):
        i=len(policies);policy.set_current_packet(current_packet);begin=time.perf_counter();action=act(observation,goal);policy_wall_s=time.perf_counter()-begin
        np.savez_compressed(out/'policy_inputs'/f'frame_{i:03d}.npz',left_rgb=observation.stereo_left_rgb,right_rgb=observation.stereo_right_rgb,proprio_history=observation.proprio_history,action_history=observation.action_history,quaternion_history=observation.quaternion_world_wxyz_history,position_world_xy=observation.position_world_xy,yaw_world_rad=observation.yaw_world_rad,pointgoal_base_xy=goal,request=action)
        policies.append(dict(frame=i,diagnostics=policy.diagnostics,request=policy.request_diagnostics,tracking=policy.tracking_diagnostics,mapping=policy.tracker.mapping_records[-1] if policy.tracker.mapping_records else None,local=policy.records[-1],policy_wall_s=policy_wall_s))
        grid=policy.grid
        np.savez_compressed(out/'maps'/f'frame_{i:03d}.npz',labels=grid.labels,provenance=grid.provenance(),floor_time=grid.memory.floor_time,hazard_time=grid.memory.hazard_time,origin=grid.origin,resolution=grid.resolution,time_s=grid.time)
        if policy.plan is not None:
            way=policy.plan.waypoints
            np.savez_compressed(out/'local_plans'/f'frame_{i:03d}.npz',pose=np.array([[p.x_m,p.y_m,p.yaw_rad] for p in way]),direction=np.array([1 if p.drive_direction=='forward' else -1 for p in way]),command=np.array([[p.main_command_rad_s,p.auxiliary_angle_rad] for p in way]),time_s=policy.reference_installed_time,version=policy.local_reference_version)
        return action
    policy.act=recorded_act
    result=None;code=1
    try:
        result=run_episode(runtime,policy,episode,action_protocol=protocol,config=RunnerConfig())
        arrays={f.name:getattr(result,f.name) for f in dataclasses.fields(result) if isinstance(getattr(result,f.name),np.ndarray)}
        np.savez_compressed(out/'episode_arrays.npz',**arrays)
        (out/'metrics.json').write_text(json.dumps(result.to_metrics_dict(),indent=2)+'\n')
        np.savez_compressed(out/'oracle_path.npz',xy=runtime.oracle_path_xy(episode))
        occ=runtime.occupancy;np.savez_compressed(out/'offline_occupancy_label.npz',labels=occ.grid,origin=occ.global_origin_xy,resolution=occ.resolution_m)
        code=0 if result.runtime_error is None else 1
        print('ROOT_DIRECTION_COMPLETE',result.termination_reason,result.elapsed_s,result.final_distance_m,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        for name,data in [('camera_clocks',clocks),('states',states),('execution_states',execution_states),('contacts',contacts),('marker_checks',markers),('physics_contacts',runtime.collision_contact_samples),('policy_records',policies)]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(completed=code==0,policy_calls=len(policies),captures=len(clocks),reset_count=1,internal_reset=False,scene_sha256=episode.scene_sha256,occupancy_sha256=episode.occupancy_sha256,camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',controller_class=str(type(policy)),runtime_class=str(type(runtime)),source_sha256={f:sha(f) for f in config['source_sha256']},global_reference_consumed=False,depth_knownpose_only=True,original_information_policy_unknown_cap_retained=False,source_model_admitted=False,nominal_geometry_only=True,extra_physics_tail=False),indent=2)+'\n')
        manifest={str(p):sha(p) for p in out.rglob('*') if p.is_file()}
        (out/'artifact_manifest.json').write_text(json.dumps(dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact_sha256=manifest,source_model_admitted=False,privileged_depth_control=True,manifest_scope="pre-runtime-close; separate post-close manifest is authoritative"),indent=2)+'\n')
        runtime.close(return_code=code)
if __name__=='__main__':main()
