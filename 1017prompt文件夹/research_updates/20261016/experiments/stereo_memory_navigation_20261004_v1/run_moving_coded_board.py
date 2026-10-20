"""One fixed twelve-second moving coded-board RGB acquisition."""
import argparse,hashlib,json,sys,traceback
from pathlib import Path
import numpy as np
from observation_schedule import schedule
from render_flush import fixed_render_flush
from capture_transaction import replace_step_images
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
from benchmark.artifacts import write_rgb_video
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
from rig_inputs import validate_capture

def main():
    protocol=json.loads((H/'moving_coded_board_protocol.json').read_text())
    out=H/'moving_coded_board_run';out.mkdir(parents=True,exist_ok=False)
    (out/'frames').mkdir()
    commands=schedule('forward');np.savez(out/'requests.npz',requests=commands)
    ref,_=load_transformed_reference(H.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
    episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
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
            if not getattr(self,'board_created',False):
                from pxr import Gf,UsdPhysics
                poses=[get_cam_pose(stage,p) for p in (LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH)]
                origin=(poses[0][:3,3]+poses[1][:3,3])/2
                plane_x=float(origin[0]+protocol['plane_ahead_of_initial_camera_m'])
                mesh=UsdGeom.Mesh.Define(stage,'/World/CalibrationOnlyMovingCodedBoard')
                from coded_board import tag_cell_corners
                points=[];indices=[];colors=[];centres=[]
                def quad(y0,z0,y1,z1,value,x):
                    start=len(points);points.extend([(x,y0,z0),(x,y1,z0),(x,y1,z1),(x,y0,z1)])
                    indices.extend([start,start+1,start+2,start+3]);colors.append((value,value,value))
                quad(origin[1]-1.12,2.26,origin[1]+1.12,.02,.95,plane_x+.002)
                codes=np.array(protocol['codes'],np.uint8)
                for tag,code in enumerate(codes):
                    r,c=divmod(tag,6);cy=origin[1]-.85+.34*c;cz=.20+.34*r
                    centres.append([plane_x,cy,cz]);bitmap=np.zeros((7,7),np.uint8);bitmap[1:6,1:6]=code
                    for rr in range(7):
                        for cc in range(7):
                            start=len(points);points.extend(tag_cell_corners([plane_x,cy,cz],rr,cc).tolist())
                            indices.extend([start,start+1,start+2,start+3]);value=.95 if bitmap[rr,cc] else .01;colors.append((value,value,value))
                mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points]);mesh.CreateFaceVertexCountsAttr([4]*len(colors));mesh.CreateFaceVertexIndicesAttr(indices)
                mesh.CreateDoubleSidedAttr(True);mesh.CreateSubdivisionSchemeAttr('none')
                display=mesh.CreateDisplayColorAttr([Gf.Vec3f(*v) for v in colors]);UsdGeom.Primvar(display).SetInterpolation('uniform')
                assert not mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI) and not mesh.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI)
                (out/'board_geometry.json').write_text(json.dumps(dict(plane_x=plane_x,centres=centres,side_m=.28,cell_m=.04,collision=False,subdivision='none',canonical_corner_permutation=[1,0,3,2],role='offline calibration labels only'),indent=2)+'\n')
                self.board_created=True
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
            states.append(dict(time_s=float(step.simulation_time_s),position=np.asarray(position).tolist(),quaternion=np.asarray(q).tolist(),velocity=np.asarray(self._robot.get_linear_velocity()).tolist(),joints=np.asarray(self._robot.get_joint_positions()).tolist(),left_pose=get_cam_pose(stage,LEFT_CAMERA_PATH).tolist(),right_pose=get_cam_pose(stage,RIGHT_CAMERA_PATH).tolist()))
            if i in protocol['selected_indices']:
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
        for command in commands:runtime.execute(command)
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        for name,frames in runtime.captured_frames.items():
            write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
        (out/'states.json').write_text(json.dumps(states,indent=2)+'\n')
        (out/'camera_clocks.json').write_text(json.dumps(clocks,indent=2)+'\n')
        sources=[Path(__file__),H/'observation_schedule.py',H/'moving_coded_board_protocol.json',H/'coded_board.py',H/'render_flush.py',H/'capture_transaction.py',P/'training/action_protocol.py',P/'scripts/action_smoothing.py',H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json',P/'scripts/camera_rig.py',Path(sys.modules['benchmark.isaac_runtime'].__file__),Path(sys.modules['roll_pid'].__file__)]
        (out/'identity.json').write_text(json.dumps(dict(name='moving_coded_board',role='actual RGB exposure calibration intervention; known poses and offline labels; not navigation',complete=True,
            actions=len(commands),captures=len(clocks),saved_frames=6,reset_count=1,internal_reset=False,acquisition='fixed3 -> singlebasecapture -> fixed3 -> imagesonly',recorded_video_frames=121,source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}),indent=2)+'\n')
        code=0;print('MOVING_CODED_BOARD_COMPLETE',len(clocks),flush=True)
    except BaseException:
        (out/'failure.txt').write_text(traceback.format_exc());raise
    finally:runtime.close(return_code=code)
if __name__=='__main__':main()
