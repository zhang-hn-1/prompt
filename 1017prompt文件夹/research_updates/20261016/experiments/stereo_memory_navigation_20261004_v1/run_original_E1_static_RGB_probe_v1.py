"""One24render original E1 static radiance probe; no NN or action."""
from pathlib import Path
import json,hashlib,numpy as np
from render_only_snapshots_v1 import record_render_only
from run_original_E1_direct_memory_navigation_v1 import IsaacBenchmarkRuntime,IsaacRuntimeConfig,DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,install_runtime,hide_diagnostic_markers,ensure_current_frames,fixed_render_flush,replace_step_images,install_sigterm_cleanup,export_then_close,write_rgb_video,camera_rig_metadata
from benchmark.manifest import BenchmarkEpisode
H=Path(__file__).resolve().parent

def main():
 install_sigterm_cleanup();cfg=json.loads((H/'original_E1_static_RGB_probe_protocol_v1.json').read_text())
 for g in ['source_sha256','input_sha256']:
  for f,s in cfg[g].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
 out=H/'original_E1_static_RGB_probe_run_v1';out.mkdir(exist_ok=False)
 manifest=json.loads(Path(cfg['episode_manifest']).read_text());episode=BenchmarkEpisode.from_dict(next(e for e in manifest['episodes'] if e['episode_id']=='golden_e1_0001'))
 class ProbeRuntime(IsaacBenchmarkRuntime):
  @staticmethod
  def _create_markers(stage,ep,path,gf,usd_geom):
   result=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom);hide_diagnostic_markers(stage,usd_geom);return result
  def frozen_state(self):return [np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
  def _capture(self):
   from collect_data_dubins import _get_rgb
   for c in (self._left_camera,self._right_camera,self._third_camera):c.set_frequency(-1)
   ensure_current_frames((self._left_camera,self._right_camera),self._clock_metadata()[2],self._world.render)
   fixed_render_flush(self._world.render,self._clock_metadata,self.frozen_state);step=super()._capture();fixed_render_flush(self._world.render,self._clock_metadata,self.frozen_state)
   return replace_step_images(step,_get_rgb(self._left_camera),_get_rgb(self._right_camera),_get_rgb(self._third_camera))
 gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json').read_text())['gains'];Runtime=install_runtime(ProbeRuntime,**gains,output=out/'feedback')
 runtime=Runtime(IsaacRuntimeConfig(scene_root=Path(cfg['scene_root']),contract_path=DEFAULT_CONTRACT.resolve(),occupancy_path=Path(cfg['occupancy']),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),gui=False,record_video=False,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',diagnostic_open_plane=False,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3));code=1
 try:
  runtime.reset(episode)
  from collect_data_dubins import _get_rgb,get_cam_pose,LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH
  from bev_ground_truth import read_camera_depth
  import omni.usd
  stage=omni.usd.get_context().get_stage();baseline_clock=runtime._clock_metadata();baseline_state=runtime.frozen_state()
  def capture():
   result={}
   for eye,camera,path in [('left',runtime._left_camera,LEFT_CAMERA_PATH),('right',runtime._right_camera,RIGHT_CAMERA_PATH)]:
    result.update({eye+'_rgb':_get_rgb(camera).copy(),eye+'_depth_m':np.asarray(read_camera_depth(camera)).copy(),eye+'_camera_usd_world':get_cam_pose(stage,path).copy(),eye+'_rendering_time':float(camera.get_current_frame()['rendering_time'])})
   return result
  rows=record_render_only(runtime._world.render,runtime._clock_metadata,runtime.frozen_state,capture,24)
  for i,r in enumerate(rows):np.savez_compressed(out/f'render_{i:02d}.npz',**r)
  write_rgb_video(out/'stereo_static_render_sequence.mp4',[np.hstack([r['left_rgb'],r['right_rgb']]) for r in rows],fps=10.)
  (out/'identity.json').write_text(json.dumps(dict(complete=True,reset_count=1,actual_render_only_snapshots=24,no_physics_after_reset=True,no_requests=True,no_NN=True,clock=baseline_clock,physical_state=[v.tolist() for v in baseline_state],camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',scene_sha256=cfg['input_sha256']['/home/an/project/NavDP/assets/scenes/benchmark_e1/e1_detour.usda'],not_original35=True,not_navigation=True,not_source_admission=True,no_seed_or_quality_setting_change=True,not_initial_state_equivalence_to_earlier_nativeIMU_collector=True),indent=2)+'\n');code=0
 except BaseException:
  import traceback
  (out/'failure.txt').write_text(traceback.format_exc());raise
 finally:
  def export():
   np.savez_compressed(out/'physics_initial.npz',**runtime.physics_diagnostics)
   (out/'camera_clocks_after.json').write_text(json.dumps(dict(clock=runtime._clock_metadata(),available_captures=len(runtime.captured_frames.get('stereo_view',[]))))+'\n')
  export_then_close(runtime,export,code)
if __name__=='__main__':main()
