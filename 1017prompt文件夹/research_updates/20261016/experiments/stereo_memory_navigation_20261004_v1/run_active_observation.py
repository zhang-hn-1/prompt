"""ONE original E1 scene complete active observation, frozen known request plan.

Full occupancy is solely the privileged acquisition guard. No visual planner.
"""
from pathlib import Path
import json,hashlib,sys,traceback
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
import scripts.run_pointgoal_benchmark as runner
from benchmark.artifacts import write_rgb_video
from scripts.camera_rig import camera_rig_metadata
from execution_snapshot import export_execution
from guarded_response import physical_state
from active_observation import admit_observation
from large_bank_schedule import schedule
from complete_capture import complete_capture
from render_flush import fixed_render_flush
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'))
from roll_pid import install_runtime
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from render_boundary import hide_diagnostic_markers,marker_visibility
sys.path.insert(0,str(H.parent/'clean_visual_hard0_intervention_20261003_v1'))
from clock_sync import ensure_current_frames
sys.path.insert(0,str(H.parent/'visible_surface_connectivity_20261004_v1'))
from rig_inputs import validate_capture,validate_mask_identity
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()

def main():
    config=json.loads((H/'active_observation_protocol.json').read_text())
    for f,s in config['source_sha256'].items():assert sha(f)==s,f
    out=H/'active_observation_run';out.mkdir(exist_ok=False)
    for name in ['online_inputs','known_poses','offline_truth','history']:(out/name).mkdir()
    commands=schedule('positive');np.savez_compressed(out/'planned_requests.npz',requests=commands)
    entry=next(e for e in json.loads((H/'guarded_response_library.json').read_text()) if e['name']=='positive')
    assert np.array_equal(commands[60:],np.asarray(entry['requests'],np.float32))
    episode=next(e for e in runner.load_manifest(config['manifest']).episodes if e.episode_id==config['episode_id'])
    occupancy=runner._resolve_occupancy_path(P/'configs/benchmark_v1/scene_index.json',episode,P/'data')
    validate_mask_identity(json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text()))
    clocks=[];states=[];execution_states=[];contacts=[];markers=[];completed=0;decision=None
    class CaptureRuntime(runner.IsaacBenchmarkRuntime):
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            result=runner.IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom);return result
        def _capture(self):
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
            np.savez_compressed(out/'offline_truth'/f'frame_{i:03d}.npz',**poses,**transforms,left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),root_position_world_m=p,root_quaternion_world_wxyz=q,time_s=t)
            if i%30==0:print('CAPTURE',i,t,flush=True)
            return step
    gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json').read_text())['gains'];Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=Path('/home/an/project/NavDP/assets/scenes'),contract_path=P/'configs/data_contract.yaml',occupancy_path=occupancy,record_video=True,gui=False,diagnostic_physics_trace=True,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2'))
    code=1
    try:
        runtime.reset(episode)
        for command in commands[:60]:
            step=runtime.execute(command);completed+=1
            if step.collision:raise RuntimeError('physical contact during settling')
        robot=runtime._robot;p,q=robot.get_world_pose();current=physical_state(p,q,robot.get_linear_velocity(),robot.get_angular_velocity(),robot.get_joint_positions(),robot.get_joint_velocities());execution=export_execution(runtime._action_smoother,runtime._action_protocol)
        occ=runtime.occupancy;grid=np.asarray(occ.grid,np.int8);decision=admit_observation(current,execution,entry,grid,occ.global_origin_xy,occ.resolution_m)
        decision['floor_height_same']=abs(float(p[2])-config['measured_cold_height_m'])<=.01;decision['admitted']=decision['admitted'] and decision['floor_height_same'];decision.update(current=current,execution=execution)
        (out/'decision.json').write_text(json.dumps(decision,indent=2)+'\n');np.savez_compressed(out/'offline_occupancy_label.npz',labels=grid,origin=occ.global_origin_xy,resolution=occ.resolution_m)
        print('LIVE_GUARD',decision['admitted'],decision['hazard_center_clearance_m'],flush=True)
        if not decision['admitted']:raise RuntimeError('active observation rejected at live current-state/sweep boundary; no motion fallback')
        assert export_execution(runtime._action_smoother,runtime._action_protocol)==execution
        for command in commands[60:]:
            step=runtime.execute(command);completed+=1
            if step.collision:raise RuntimeError('physical contact; sequence aborted without second action')
        code=0;print('ACTIVE_OBSERVATION_COMPLETE',completed,len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:
        np.savez_compressed(out/'requests.npz',requests=commands[:completed]);np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        for name,data in [('camera_clocks',clocks),('states',states),('execution_states',execution_states),('contacts',contacts),('marker_checks',markers),('physics_contacts',runtime.collision_contact_samples)]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        (out/'identity.json').write_text(json.dumps(dict(completed=code==0,requests=completed,captures=len(clocks),reset_count=1,internal_reset=False,scene_sha256=episode.scene_sha256,occupancy_sha256=episode.occupancy_sha256,camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',controller_class=str(type(runtime)),admission_role='known-map2D acquisition only, no visual guarantee',source_sha256={f:sha(f) for f in config['source_sha256']}),indent=2)+'\n')
        runtime.close(return_code=code)
if __name__=='__main__':main()
