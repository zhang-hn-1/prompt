"""One complete32s reverse-straight domain sequence with fixed current rig."""
import argparse,hashlib,json,sys,traceback,dataclasses
from pathlib import Path
import numpy as np
from reverse_half_schedule_v1 import schedule
from probe_cleanup import install_sigterm_cleanup
from reverse_half_development_admission_v1 import require_development_point
from execution_snapshot import export_execution
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
    parser=argparse.ArgumentParser();parser.add_argument('--arm',choices=['development','validation'],required=True);a=parser.parse_args();a.name='reverse_half_'+a.arm
    config=json.loads((H/'reverse_half_protocol_v1.json').read_text())
    for file,digest in config['source_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    for file,digest in config['input_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    out=H/('reverse_half_'+a.arm+'_run_v1');out.mkdir(parents=True,exist_ok=False)
    (out/'frames').mkdir()
    commands=schedule();np.savez(out/'requests.npz',requests=commands)
    ref,_=load_transformed_reference(H.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
    episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
    yaw=np.radians(config['arms'][a.arm]['initial_yaw_deg']);episode=dataclasses.replace(episode,start_quaternion_world_wxyz=(float(np.cos(yaw/2)),0.,0.,float(np.sin(yaw/2))))
    if a.arm=='validation':
        point=require_development_point(H,config)
    clocks=[];states=[]
    class CaptureRuntime(IsaacBenchmarkRuntime):
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            r=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom);return r
        def _capture(self):
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
                np.savez_compressed(out/'frames'/f'frame_{i:03d}.npz',**rgb,**transforms,
                    left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),
                    left_camera_usd_world=get_cam_pose(stage,LEFT_CAMERA_PATH),right_camera_usd_world=get_cam_pose(stage,RIGHT_CAMERA_PATH),
                    root_position_world_m=position,root_quaternion_world_wxyz=q,time_s=step.simulation_time_s)
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(IsaacRuntimeConfig(scene_root=DEFAULT_SCENE_ROOT.resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,
        record_video=True,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),
        lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',diagnostic_open_plane=True,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3))
    code=1
    try:
        runtime.set_reference_visualization_path(ref.points_xy)
        runtime.reset(episode)
        initial=export_execution(runtime._action_smoother,runtime._action_protocol)
        assert initial['pid_config']==config['pid_config'] and initial['inner_config']==config['inner_config']
        (out/'initial_execution_after_reset.json').write_text(json.dumps(initial,indent=2)+'\n')
        for command in commands:runtime.execute(command)
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():
            write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        (out/'states.json').write_text(json.dumps(states,indent=2)+'\n')
        (out/'camera_clocks.json').write_text(json.dumps(clocks,indent=2)+'\n')
        (out/'physics_contacts.json').write_text(json.dumps(runtime.collision_contact_samples,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(name=a.name,role='one32s current reverse-straight point diagnostic, no navigation or fit',complete=True,actions=len(commands),captures=len(clocks),saved_frames=65,reset_count=1,internal_reset=False,recorded_video_frames=321,acquisition='fixed3 -> singlebasecapture -> fixed3 -> imagesonly',camera_rig=camera_rig_metadata(),pid_config=initial['pid_config'],inner_config=initial['inner_config'],runtime_class=str(type(runtime)),pid_module_file=str(sys.modules['roll_pid'].__file__),source_sha256={file:hashlib.sha256(Path(file).read_bytes()).hexdigest() for file in config['source_sha256']},input_sha256=config['input_sha256'],no_model_fit=True,no_controller_tuning=True,extra_physics_tail=False),indent=2)+'\n')
        code=0;print('OBSERVATION_COMPLETE',a.name,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        try:
            if code!=0:
                errors=[]
                exports=[('states.json',lambda:(out/'states.json').write_text(json.dumps(states,indent=2)+'\n')),
                         ('camera_clocks.json',lambda:(out/'camera_clocks.json').write_text(json.dumps(clocks,indent=2)+'\n')),
                         ('physics.npz',lambda:np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)),
                         ('physics_contacts.json',lambda:(out/'physics_contacts.json').write_text(json.dumps(runtime.collision_contact_samples,indent=2)+'\n'))]
                for label,export in exports:
                    try:export()
                    except BaseException as e:errors.append(dict(file=label,error=repr(e)))
                (out/'partial_export_status.json').write_text(json.dumps(dict(complete=False,errors=errors),indent=2)+'\n')
        finally:runtime.close(return_code=code)
if __name__=='__main__':main()
