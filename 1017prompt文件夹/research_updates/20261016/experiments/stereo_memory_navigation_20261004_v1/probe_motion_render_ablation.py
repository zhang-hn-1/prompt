"""One frozen forward-stop interface probe; no policy, fitting or score changes."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
from observation_schedule import schedule
from render_flush import fixed_render_flush
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from scripts.run_round4_fixed_reference import load_transformed_reference
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'))
from roll_pid import install_runtime
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from render_boundary import hide_diagnostic_markers
out=H/'motion_render_ablation_probe';out.mkdir(exist_ok=False);(out/'frames').mkdir()
protocol=dict(role='One forward-stop acquisition interface diagnostic, not navigation/fresh model validation',requested=schedule('forward').tolist(),duration_s=12,reset_only_at_start=True,flush_renders=3,comparison='Three plain renders A, then three additional plain renders B, then three FK-refreshed renders C; same physical state; isolate render-count versus FK attribution',selected_indices=[0,4,60,70,80,90,100,110,120],rig_unchanged=True,control_unchanged=True)
(out/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
ref,_=load_transformed_reference(H.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
records=[]
class Probe(IsaacBenchmarkRuntime):
    @staticmethod
    def _create_markers(stage,ep,path,gf,usd_geom):
        r=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom);hide_diagnostic_markers(stage,usd_geom);return r
    def snapshot(self):
        return [np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
    def _capture(self):
        from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose,_get_rgb
        from bev_ground_truth import read_camera_depth
        import omni.usd
        for camera in (self._left_camera,self._right_camera):camera.set_frequency(-1)
        fixed_render_flush(self._world.render,self._clock_metadata,self.snapshot)
        step=super()._capture();i=len(records);stage=omni.usd.get_context().get_stage();row=dict(index=i,time_s=float(step.simulation_time_s),raw_time_s=float(step.raw_simulation_time_s),eyes={})
        data=dict(root_position=self._robot.get_world_pose()[0],root_quaternion=self._robot.get_world_pose()[1],time_s=step.simulation_time_s)
        for eye,camera,path in (('left',self._left_camera,LEFT_CAMERA_PATH),('right',self._right_camera,RIGHT_CAMERA_PATH)):
            cached=np.asarray(read_camera_depth(camera)).copy();direct=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data()).copy()
            rgb_cached=_get_rgb(camera).copy();rgb_direct=camera.get_rgb().copy();both=np.isfinite(cached)&np.isfinite(direct)
            row['eyes'][eye]=dict(timestamp_age_s=float(step.raw_simulation_time_s)-float(camera.get_current_frame()['rendering_time']),depth_finite_pattern_equal=bool(np.array_equal(np.isfinite(cached),np.isfinite(direct))),depth_max_difference_m=float(np.max(abs(cached[both]-direct[both]))) if both.any() else None,rgb_mean_difference=float(np.mean(abs(rgb_cached.astype(float)-rgb_direct))))
            data.update({eye+'_cached_depth':cached,eye+'_direct_depth':direct,eye+'_cached_rgb':rgb_cached,eye+'_direct_rgb':rgb_direct,eye+'_pose':get_cam_pose(stage,path)})
        fixed_render_flush(self._world.render,self._clock_metadata,self.snapshot)
        for eye,camera in (('left',self._left_camera),('right',self._right_camera)):
            data[eye+'_extra_plain_depth']=np.asarray(read_camera_depth(camera)).copy()
            data[eye+'_extra_plain_rgb']=_get_rgb(camera).copy()
        def sync_render():
            self._world.physics_sim_view.update_articulations_kinematic()
            self._world.render()
        fixed_render_flush(sync_render,self._clock_metadata,self.snapshot)
        for eye,camera in (('left',self._left_camera),('right',self._right_camera)):
            data[eye+'_synced_depth']=np.asarray(read_camera_depth(camera)).copy()
            data[eye+'_synced_rgb']=_get_rgb(camera).copy()
        records.append(row)
        if i in protocol['selected_indices']:np.savez_compressed(out/'frames'/f'frame_{i:03d}.npz',**data)
        return step
gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json').read_text())['gains'];Runtime=install_runtime(Probe,**gains,output=out/'feedback')
runtime=Runtime(IsaacRuntimeConfig(scene_root=DEFAULT_SCENE_ROOT.resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,record_video=False,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',diagnostic_open_plane=True,diagnostic_physics_trace=True))
runtime.set_reference_visualization_path(ref.points_xy);code=1
try:
    runtime.reset(episode)
    for action in schedule('forward'):runtime.execute(action)
    np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
    (out/'records.json').write_text(json.dumps(records,indent=2)+'\n')
    paths=[Path(__file__),out/'protocol.json',H/'render_flush.py',H/'observation_schedule.py',Path(sys.modules['benchmark.isaac_runtime'].__file__),Path(sys.modules['roll_pid'].__file__)]
    (out/'identity.json').write_text(json.dumps(dict(complete=True,actions=120,captures=len(records),source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n');code=0
    print('MOTION_RENDER_ABLATION_PROBE_COMPLETE',len(records),flush=True)
finally:runtime.close(return_code=code)
