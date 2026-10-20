"""Re-execute frozen development requests in real physics with current cameras."""
import argparse,json,hashlib,sys,traceback
from dataclasses import replace
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
BASELINE=HERE.parent/'visual_information_new_rig_20261004_v2'
from render_flush import fixed_render_flush
from complete_capture import complete_capture
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(PROJECT))
import scripts.run_pointgoal_benchmark as runner
from scripts.camera_rig import camera_rig_metadata
sys.path.insert(0,str(BASELINE))
from rig_inputs import validate_capture,validate_mask_identity
BOUNDARY=HERE.parent/'bilateral_temporal_geometry_navigation_20261003_v1'
sys.path.insert(0,str(BOUNDARY));from render_boundary import hide_diagnostic_markers,marker_visibility
sys.path.insert(0,str(HERE.parent/'clean_visual_hard0_intervention_20261003_v1'));from clock_sync import ensure_current_frames
ROLL=HERE.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'
sys.path.insert(0,str(ROLL));from roll_pid import install_runtime


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--sequence',required=True);args=parser.parse_args()
    row=next(r for r in json.loads((HERE/'transaction_development_protocol.json').read_text())['sequences'] if r['sequence']==args.sequence)
    source=Path(row['requests_file']);assert hashlib.sha256(source.read_bytes()).hexdigest()==row['requests_sha256']
    requests=np.load(source)['requests'];selected=set(row['selected_indices'])
    episode=next(e for e in runner.load_manifest(row['manifest']).episodes if e.episode_id==row['episode_id'])
    out=HERE/'capture_transaction_development'/args.sequence;assert not out.exists();out.mkdir(parents=True)
    validate_mask_identity(json.loads((BASELINE/'mask_identity.json').read_text()))
    for folder in ('online_inputs','offline_truth'):(out/folder).mkdir(exist_ok=True)
    clocks=[];positions=[];saved=[];all_markers=[]
    class CaptureRuntime(runner.IsaacBenchmarkRuntime):
        def __init__(self,config):
            super().__init__(config);self.capture_index=0;self.acquisition_enabled=False
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
            if not self.acquisition_enabled:
                for camera in (self._left_camera,self._right_camera):camera.set_frequency(-1)
                self.acquisition_enabled=True
            clock=self._clock_metadata()
            before=[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            ensure_current_frames((self._left_camera,self._right_camera),clock[2],self._world.render)
            fixed_render_flush(self._world.render,self._clock_metadata,lambda:[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())])
            after=[np.asarray(v) for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            assert clock==self._clock_metadata() and all(np.array_equal(a,b) for a,b in zip(before,after))
            visibility=marker_visibility(stage,UsdGeom);assert all(v['computed_visibility']=='invisible' for v in visibility.values())
            all_markers.append(visibility)
            before_depths={eye+'_pretransaction_depth_m':read_camera_depth(camera).copy() for eye,camera in [('left',self._left_camera),('right',self._right_camera)]}
            base_capture=super()._capture
            step=complete_capture(base_capture,self._world.render,self._clock_metadata,
                lambda:[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())],
                lambda:(_get_rgb(self._left_camera),_get_rgb(self._right_camera),_get_rgb(self._third_camera)))
            i=self.capture_index;self.capture_index+=1
            raw=float(step.raw_simulation_time_s)
            times={eye:float(camera.get_current_frame()['rendering_time']) for eye,camera in [('left',self._left_camera),('right',self._right_camera)]}
            assert all(abs(raw-t)<=1e-5 for t in times.values());clocks.append(dict(frame=i,raw_physics_time_s=raw,episode_time_s=float(step.simulation_time_s),eyes=times))
            positions.append(np.asarray(self._robot.get_world_pose()[0]).copy())
            if i in selected:
                rgb={eye+'_rgb':_get_rgb(camera) for eye,camera in [('left',self._left_camera),('right',self._right_camera)]}
                transforms={eye+'_optical_to_base':_camera_optical_to_base(stage,path) for eye,path in [('left',LEFT_CAMERA_PATH),('right',RIGHT_CAMERA_PATH)]}
                validate_capture(dict(**rgb,**transforms,raw_time_s=raw,left_render_time_s=times['left'],right_render_time_s=times['right']))
                obs=step.observation
                np.savez_compressed(out/'online_inputs'/f'frame_{i:03d}.npz',**rgb,position_world_xy=obs.position_world_xy,yaw_world_rad=obs.yaw_world_rad,
                    quaternion_world_wxyz_history=obs.quaternion_world_wxyz_history,proprio_history=obs.proprio_history,action_history=obs.action_history,
                    simulation_time_s=step.simulation_time_s)
                p,q=self._robot.get_world_pose()
                np.savez_compressed(out/'offline_truth'/f'frame_{i:03d}.npz',**transforms,**before_depths,left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),
                    left_camera_usd_world=get_cam_pose(stage,LEFT_CAMERA_PATH),right_camera_usd_world=get_cam_pose(stage,RIGHT_CAMERA_PATH),root_position_world_m=p,root_quaternion_world_wxyz=q,
                    simulation_time_s=step.simulation_time_s)
                saved.append(i)
            return step
    gains=json.loads((ROLL/'selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
    occupancy=runner._resolve_occupancy_path(PROJECT/'configs/benchmark_v1/scene_index.json',episode,PROJECT/'data')
    runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=Path('/home/an/project/NavDP/assets/scenes'),contract_path=PROJECT/'configs/data_contract.yaml',gui=False,
        record_video=False,collision_mode='physical_contact_v2',lighting_profile='fixed_camera_fill_v1',occupancy_path=occupancy,diagnostic_physics_trace=True))
    try:
        runtime.reset(episode)
        for action in requests:runtime.execute(action)
        assert saved==row['selected_indices'],(saved,row['selected_indices'])
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        np.savez_compressed(out/'root_positions.npz',position_world_m=np.stack(positions))
        (out/'camera_clocks.json').write_text(json.dumps(clocks,indent=2)+'\n')
        (out/'markers.json').write_text(json.dumps(all_markers)+'\n')
        # Retrospective trajectory equivalence check; never an input to simulation/model.
        old=Path(row['source_root']);diff={}
        for i in saved:
            a=np.load(old/'online_inputs'/f'frame_{i:03d}.npz');b=np.load(out/'online_inputs'/f'frame_{i:03d}.npz')
            for key in ('position_world_xy','yaw_world_rad','quaternion_world_wxyz_history','proprio_history','action_history'):
                diff[key]=max(diff.get(key,0),float(np.max(np.abs(a[key]-b[key]))))
        (out/'capture_summary.json').write_text(json.dumps(dict(completed=True,role='single base capture then protected fixed3render image transaction; targeted development replays; not fresh held-out/navigation',
            rig=camera_rig_metadata(),sequence=args.sequence,request_count=len(requests),selected_indices=saved,physics_steps=int(runtime.physics_diagnostics['simulation_time_s'].size),
            old_new_state_max_abs_differences=diff,render_preserved_clock_and_state=True,all_markers_hidden=True,source_request_sha256=row['requests_sha256'],
            source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(runner.__file__),Path(sys.modules['benchmark.isaac_runtime'].__file__),PROJECT/'scripts/camera_rig.py',ROLL/'roll_pid.py',BASELINE/'rig_inputs.py',HERE/'render_flush.py',HERE/'complete_capture.py',HERE/'capture_transaction.py',HERE/'transaction_development_protocol.json']}),indent=2)+'\n')
        print('CAPTURE_COMPLETE',args.sequence,len(saved),diff,flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True);runtime.close(return_code=1);raise
    finally:runtime.close()

if __name__=='__main__':main()
