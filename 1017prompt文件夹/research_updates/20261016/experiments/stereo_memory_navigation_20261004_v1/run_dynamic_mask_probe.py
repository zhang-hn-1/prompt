"""One separately-labelled local2m RGB-knownpose risk experiment per arm."""
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
from fast_stereo import FastStereo
from dense_source_points import point_packet
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
    config=json.loads((H/'dynamic_mask_probe_protocol.json').read_text())
    for f,s in config['source_sha256'].items():assert sha(f)==s,f
    arm=sys.argv[1];assert arm in config['arms'];out=H/'dynamic_mask_probe_run';out.mkdir(exist_ok=False)
    for name in ['online_inputs','known_poses','offline_truth','history','sensor_packets','source_predictions','maps','local_plans','policy_inputs','self_validation']:(out/name).mkdir()
    episode=next(e for e in runner.load_manifest(config['manifest']).episodes if e.episode_id==config['episode_id'])
    occupancy=runner._resolve_occupancy_path(P/'configs/benchmark_v1/scene_index.json',episode,P/'data')
    original_episode=episode
    episode=dataclasses.replace(episode,episode_id='dynamic_mask_probe_12s')
    (out/'task_episode.json').write_text(json.dumps(dict(task='predeclared selfocclusion diagnostic12s, no navigation claim',original_episode=original_episode.to_dict(),actual_episode=episode.to_dict()),indent=2)+'\n')
    validate_mask_identity(json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text()))
    maskdata=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    yy,xx=np.mgrid[0:480:4,0:640:4]
    rays=np.column_stack(((xx.ravel()-319.5)/584.8888888888889,(yy.ravel()-239.5)/584.8888888888889,np.ones(xx.size)))
    clocks=[];states=[];execution_states=[];contacts=[];markers=[];policies=[];current_packet=None;source_packet=None;depth_model=None
    class CaptureRuntime(runner.IsaacBenchmarkRuntime):
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            result=runner.IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom);return result
        def _capture(self):
            nonlocal current_packet,source_packet,depth_model
            capture_started=time.perf_counter()
            import omni.usd
            from pxr import UsdGeom
            from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose,_camera_optical_to_base,_get_rgb
            from bev_ground_truth import read_camera_depth
            stage=omni.usd.get_context().get_stage()
            if not getattr(self,'fresh_cameras',False):
                for camera in (self._left_camera,self._right_camera,self._third_camera):camera.set_frequency(-1)
                self.fresh_cameras=True
                from isaacsim.core.utils.semantics import add_update_semantics
                from pxr import Usd
                robot_root=stage.GetPrimAtPath('/World/BallRobot')
                for prim in Usd.PrimRange(robot_root):
                    if prim.IsA(UsdGeom.Mesh):add_update_semantics(prim,'robot_self_validation_only')
                for camera in (self._left_camera,self._right_camera):camera.add_instance_id_segmentation_to_frame()
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
            # No model/depth control: source packet is an explicit empty placeholder.
            source_packet=dict(world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,int),partner_pixel_id=np.empty(0,int),frame=i,time_s=t)
            current_packet=dict(frame=i,time_s=t,points_world=np.empty((0,3)),root_xyz=np.asarray(p))
            clocks[-1].update(rgb_source_wall_s=0.,source_predicted=False,source_frame=i,source_time_s=t,source_age_s=0.)
            if i in (0,30,60,120):
                # Offline validation only. Neither IDs nor poses enter the fixed request program.
                from collect_data_dubins import BASE_ROOT,ROBOT_ROOT
                from pxr import UsdGeom
                cache=UsdGeom.XformCache();body={name+'_usd_world':np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT_ROOT+'/'+name))).T for name in ('base_link','link1','link2','link3')}
                data=dict(**body,**poses,joint_position_rad=np.asarray(self._robot.get_joint_positions()),proprio_history=step.observation.proprio_history,time_s=t,raw_time_s=rawtime)
                metadata=dict(frame=i,time_s=t,joint_names=list(self._joint_names(self._robot)),segmentation={})
                for eye,cam in [('left',self._left_camera),('right',self._right_camera)]:
                    seg=cam.get_current_frame().get('instance_id_segmentation')
                    if isinstance(seg,dict):
                        data[eye+'_instance_ids']=np.asarray(seg['data']);metadata['segmentation'][eye]=dict(info=seg.get('info',{}),mapping_available=bool(seg.get('info',{})))
                    elif seg is not None:
                        data[eye+'_instance_ids']=np.asarray(seg);metadata['segmentation'][eye]=dict(mapping_available=False)
                    else:metadata['segmentation'][eye]=dict(mapping_available=False,data_missing=True)
                np.savez_compressed(out/'self_validation'/f'frame_{i:03d}.npz',**data)
                (out/'self_validation'/f'frame_{i:03d}.json').write_text(json.dumps(metadata,indent=2,default=str)+'\n')
            clocks[-1]['capture_partial_wall_s']=time.perf_counter()-capture_started
            np.savez_compressed(out/'offline_truth'/f'frame_{i:03d}.npz',**poses,**transforms,**depths,root_position_world_m=p,root_quaternion_world_wxyz=q,time_s=t)
            if i%30==0:print('CAPTURE',i,t,flush=True)
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=Path('/home/an/project/NavDP/assets/scenes'),contract_path=P/'configs/data_contract.yaml',occupancy_path=occupancy,record_video=True,gui=False,diagnostic_physics_trace=True,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',auxiliary_velocity_limit_rad_s=.3))
    protocol=runner.load_action_protocol(P/'configs/data_contract.yaml')
    class ProgramPolicy:
        policy_name='predeclared12s_self_occlusion_probe_no_navigation'
        def reset(self,context):self.frame=0
        def act(self,observation,goal):
            t=self.frame*.1;main=0. if t<3. or t>=9. else .3;aux=0. if t<3. or t>=9. else (.15 if t<6. else -.15);action=np.tile(np.array([main,aux],np.float32),5);policies.append(dict(frame=self.frame,time_s=t,request=action.tolist(),predeclared=True,no_sensor_decision=True));self.frame+=1;return action
    policy=ProgramPolicy()
    result=None;code=1
    try:
        result=run_episode(runtime,policy,episode,action_protocol=protocol,config=RunnerConfig(episode_budget_s=12.))
        arrays={f.name:getattr(result,f.name) for f in dataclasses.fields(result) if isinstance(getattr(result,f.name),np.ndarray)}
        np.savez_compressed(out/'episode_arrays.npz',**arrays)
        (out/'metrics.json').write_text(json.dumps(result.to_metrics_dict(),indent=2)+'\n')
        np.savez_compressed(out/'oracle_path.npz',xy=runtime.oracle_path_xy(episode))
        occ=runtime.occupancy;np.savez_compressed(out/'offline_occupancy_label.npz',labels=occ.grid,origin=occ.global_origin_xy,resolution=occ.resolution_m)
        code=0 if result.runtime_error is None else 1
        print('DYNAMIC_MASK_PROBE_COMPLETE',arm,result.termination_reason,result.elapsed_s,result.final_distance_m,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        for name,data in [('camera_clocks',clocks),('states',states),('execution_states',execution_states),('contacts',contacts),('marker_checks',markers),('physics_contacts',runtime.collision_contact_samples),('policy_records',policies)]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(completed=code==0,policy_calls=len(policies),diagnostic_only=True,validation_capture_frames=[0,30,60,120],model_inferences=0,captures=len(clocks),reset_count=1,internal_reset=False,scene_sha256=episode.scene_sha256,occupancy_sha256=episode.occupancy_sha256,camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',controller_class=str(type(policy)),runtime_class=str(type(runtime)),source_sha256={f:sha(f) for f in config['source_sha256']},global_reference_consumed=False,rgb_depth_knownpose=False,source_model_admitted_in_old_domain=False,task_prior_is_observation=False,local_goal_m=None,memory_arm=arm,shared_unknown_request_main_cap_rad_s=.3,formal_metrics_not_applicable=True,source_model_admitted=False,nominal_geometry_only=True,extra_physics_tail=False),indent=2)+'\n')
        manifest={str(p):sha(p) for p in out.rglob('*') if p.is_file()}
        (out/'artifact_manifest.json').write_text(json.dumps(dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact_sha256=manifest,source_model_admitted=False,rgb_control=False,knownpose_privileged=True,task_prior_is_observation=False,not_formal_navigation=True,manifest_scope="pre-runtime-close; separate post-close manifest is authoritative"),indent=2)+'\n')
        runtime.close(return_code=code)
if __name__=='__main__':main()
