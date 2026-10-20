"""One preregistered stop-bank request intervention; frozen capture implementation reused."""
import argparse,hashlib,json,sys,traceback
from dataclasses import replace
from execution_snapshot import export_execution
from guarded_response import GuardedResponseConsumer,physical_state
from spatial_memory import SpatialMemory
from pathlib import Path
import numpy as np
from short_branch import development_schedule
from stop_bank_hold import hold_bank_schedule
from render_flush import fixed_render_flush
from capture_transaction import replace_step_images
from benchmark.artifacts import write_rgb_video
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from scripts.run_round4_fixed_reference import load_transformed_reference
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'))
from roll_pid import install_runtime
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from render_boundary import hide_diagnostic_markers,marker_visibility
sys.path.insert(0,str(H.parent/'clean_visual_hard0_intervention_20261003_v1'))
from clock_sync import ensure_current_frames
sys.path.insert(0,str(H.parent/'visible_surface_connectivity_20261004_v1'))
from rig_inputs import validate_capture,validate_mask_identity

def main():
    config=json.loads((H/'stop_bank_hold_protocol.json').read_text())
    out=H/'stop_bank_hold_run';out.mkdir(parents=True,exist_ok=False)
    (out/'frames').mkdir()
    commands=hold_bank_schedule(development_schedule()[:75])
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    validate_mask_identity(json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text()))
    yy,xx=np.mgrid[0:480:4,0:640:4];focal=16.45/18*640
    rays=np.column_stack(((xx.ravel()-319.5)/focal,(yy.ravel()-239.5)/focal,np.ones(xx.size)))
    memory=None;decisions=[];execution_states=[];completion_records=[]
    ref,_=load_transformed_reference(H.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
    episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
    original_yaw=2*np.arctan2(episode.start_quaternion_world_wxyz[3],episode.start_quaternion_world_wxyz[0])
    clocks=[];states=[]
    class CaptureRuntime(IsaacBenchmarkRuntime):
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            r=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom);return r
        def _capture(self):
            nonlocal memory
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
            history=out/'history';history.mkdir(exist_ok=True)
            np.savez_compressed(history/f'frame_{i:03d}.npz',proprio_history=step.observation.proprio_history,action_history=step.observation.action_history,quaternion_world_wxyz_history=step.observation.quaternion_world_wxyz_history)
            times={eye:float(camera.get_current_frame()['rendering_time']) for eye,camera in [('left',self._left_camera),('right',self._right_camera)]}
            assert all(abs(float(step.raw_simulation_time_s)-t)<=1e-5 for t in times.values())
            clocks.append(dict(frame=i,raw_time_s=float(step.raw_simulation_time_s),episode_time_s=float(step.simulation_time_s),eyes=times,markers_hidden=True))
            position,q=self._robot.get_world_pose()
            snapshot=export_execution(self._action_smoother,self._action_protocol) if hasattr(self._action_smoother,'pid') else None
            execution_states.append(snapshot)
            packet=[]
            for eye,camera,path in [('left',self._left_camera,LEFT_CAMERA_PATH),('right',self._right_camera,RIGHT_CAMERA_PATH)]:
                depth=read_camera_depth(camera)[::4,::4].ravel();pose=get_cam_pose(stage,path)
                rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
                valid=np.isfinite(depth)&(depth>=.45)&(depth<=8.)&(~masks[eye][::4,::4].ravel())
                packet.append(pose[:3,3]+(rays[valid]*depth[valid,None])@rotation.T)
            points=np.concatenate(packet);points=points[np.linalg.norm(points-position,axis=1)>=.55]
            if memory is None:memory=SpatialMemory(np.asarray(position)[:2]-20,400,.1,max_age_s=10.)
            memory.update(points,float(step.simulation_time_s),pose_sigma_m=0.)
            packets=out/'packets';packets.mkdir(exist_ok=True)
            np.savez_compressed(packets/f'frame_{i:03d}.npz',points_world=points,root_xyz=position,time_s=step.simulation_time_s)
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
        lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',diagnostic_open_plane=True,diagnostic_physics_trace=True))
    runtime.set_reference_visualization_path(ref.points_xy)
    code=1
    try:
        runtime.reset(episode)
        for command in commands[:60]:runtime.execute(command)
        def convert(v):
            if isinstance(v,np.ndarray):return v.tolist()
            raise TypeError(type(v))
        for command in commands[60:]:runtime.execute(command)
        (out/'runtime_configuration.json').write_text(json.dumps(runtime.diagnostic_configuration,default=convert,indent=2)+'\n')
        (out/'contacts.json').write_text(json.dumps(runtime.collision_contact_samples,default=convert,indent=2)+'\n')
        np.savez_compressed(out/'requests.npz',requests=commands)
        (out/'completion_records.json').write_text(json.dumps(completion_records,indent=2)+'\n')
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():
            write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        (out/'execution_states.json').write_text(json.dumps(execution_states,indent=2)+'\n')
        (out/'states.json').write_text(json.dumps(states,indent=2)+'\n')
        (out/'camera_clocks.json').write_text(json.dumps(clocks,indent=2)+'\n')
        sources=[Path(__file__),H/'guarded_response.py',H/'execution_snapshot.py',H/'short_branch.py',H/'stop_bank_hold.py',H/'run_short_branch.py',H/'stop_bank_hold_protocol.json',H/'spatial_memory.py',H/'primitive_planner.py',H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz',H.parent/'visible_surface_connectivity_20261004_v1/rig_inputs.py',H.parent/'bilateral_temporal_geometry_navigation_20261003_v1/render_boundary.py',H.parent/'clean_visual_hard0_intervention_20261003_v1/clock_sync.py',H/'render_flush.py',H/'capture_transaction.py',P/'training/action_protocol.py',P/'scripts/action_smoothing.py',H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json',P/'scripts/camera_rig.py',Path(sys.modules['benchmark.isaac_runtime'].__file__),Path(sys.modules['roll_pid'].__file__)]
        sources.extend([DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY])
        (out/'identity.json').write_text(json.dumps(dict(name='stop_bank_hold',role='single controlled hold-prior-bank while main stops; privileged depth/pose; not navigation',complete=True,
            actions=len(commands),captures=len(clocks),saved_frames=len(commands)//5+1,reset_count=1,internal_reset=False,acquisition='fixed3 -> singlebasecapture -> fixed3 -> imagesonly',recorded_video_frames=len(commands)+1,source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}),indent=2)+'\n')
        code=0;print('STOP_BANK_HOLD_COMPLETE',len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:runtime.close(return_code=code)
if __name__=='__main__':main()
