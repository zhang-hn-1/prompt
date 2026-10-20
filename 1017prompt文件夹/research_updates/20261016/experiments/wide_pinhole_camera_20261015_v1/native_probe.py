from pathlib import Path
import sys,json,hashlib,numpy as np,traceback
D=Path(__file__).resolve().parent;W=D.parents[1];C=D.parent/'camera_exposure_contract_20261015_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(C),str(D.parent/'stereo_memory_navigation_20261004_v1'),str(P)]
from run_original_E1_direct_memory_navigation_v1 import IsaacBenchmarkRuntime,IsaacRuntimeConfig,DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,hide_diagnostic_markers,install_sigterm_cleanup,export_then_close
from benchmark.manifest import BenchmarkEpisode
from exposure_contract import install_preinitialized_camera,attach_camera_parameters,paired_snapshot,local_transform
from imu_interval_packet import interval_packet
cfg=json.loads((D/'protocol.json').read_text());output_name=sys.argv[1] if len(sys.argv)>1 else 'native_recovery_run';assert output_name in ('native_recovery_run','native_validation_run');out=D/output_name;out.mkdir(exist_ok=False);(out/'frames').mkdir();imu={};rows=[];code=1
class Runtime(IsaacBenchmarkRuntime):
 def _initialize(self,*args,**kwargs):
  import isaacsim
  App=isaacsim.SimulationApp
  def prepared_app(*aa,**kk):
   app=App(*aa,**kk);self._simulation_app=app
   if str(P/'scripts') not in sys.path:sys.path.insert(0,str(P/'scripts'))
   import omni.isaac.sensor as module
   from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH
   specs={p:dict(position=pos,yaw=yaw,focal_mm=cfg['focal_mm']) for p,pos,yaw in zip([LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH],cfg['positions_m'],cfg['yaw_degrees'])}
   self.research_sensor_module=module;self.research_original_camera=install_preinitialized_camera(module,specs)
   return app
  isaacsim.SimulationApp=prepared_app
  try:return super()._initialize(*args,**kwargs)
  finally:
   isaacsim.SimulationApp=App
   if hasattr(self,'research_sensor_module'):self.research_sensor_module.Camera=self.research_original_camera
 @staticmethod
 def _create_markers(stage,ep,path,gf,usd_geom):
  result=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom);hide_diagnostic_markers(stage,usd_geom)
  import omni.kit.commands
  from isaacsim.sensors.physics import _sensor
  from collect_data_dubins import BASE_ROOT
  path=BASE_ROOT+'/wide_native_imu';ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/wide_native_imu',parent=BASE_ROOT,sensor_period=.02,translation=gf.Vec3d(0,0,0),orientation=gf.Quatd(1,0,0,0),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1);assert ok and prim is not None;imu.update(path=path,interface=_sensor.acquire_imu_sensor_interface());return result
runtime=Runtime(IsaacRuntimeConfig(scene_root=Path(cfg['scene_root']),contract_path=DEFAULT_CONTRACT.resolve(),occupancy_path=Path(cfg['occupancy']),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),gui=False,record_video=False,lighting_profile=cfg['lighting_profile'],collision_mode='physical_contact_v2',diagnostic_open_plane=False,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3))
try:
 ep=BenchmarkEpisode.from_dict(next(e for e in json.loads(Path(cfg['episode_manifest']).read_text())['episodes'] if e['episode_id']=='golden_e1_0001'));runtime.reset(ep)
 import omni.usd
 from pxr import UsdGeom
 from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose
 cams=[runtime._left_camera,runtime._right_camera]
 for c in cams:attach_camera_parameters(c);c.set_frequency(-1)
 stage=omni.usd.get_context().get_stage();stage.GetRootLayer().Export(str(out/'stage_root_initial.usda'));used=[]
 for l in stage.GetUsedLayers():
  path=Path(l.realPath) if l.realPath else None;used.append(dict(identifier=l.identifier,real_path=str(path) if path else None,sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path and path.is_file() else hashlib.sha256(l.ExportToString().encode()).hexdigest()))
 (out/'stage_identity.json').write_text(json.dumps(dict(used_layers=used,meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)),lighting=cfg['lighting_profile'],lamp_positions=runtime.lighting_lamp_positions_world_m),indent=2)+'\n')
 previous=[-1,-1];relative=np.linalg.inv(local_transform(cfg['positions_m'][0],0))@local_transform(cfg['positions_m'][1],180)
 for i in range(8):
  for unused in range(5):runtime.execute(np.zeros(10,np.float32))
  clock=runtime._clock_metadata();runtime._world.render();runtime._world.render();assert runtime._clock_metadata()==clock
  packets=paired_snapshot(cams,previous,float(clock[2]),cfg['focal_px']);previous=[p['render_frame'] for p in packets];actual=np.linalg.inv(packets[0]['camera_usd_world_label'])@packets[1]['camera_usd_world_label'];pair_error=float(np.max(abs(actual-relative)));assert pair_error<=2e-5,('renderer relative camera calibration',pair_error)
  diag=runtime.physics_diagnostics;raw=diag['raw_simulation_time_s'];reading=imu['interface'].get_sensor_reading(imu['path'],use_latest_data=True,read_gravity=True);_,imeta=interval_packet(reading,float(raw[-2]),float(raw[-1]),float(clock[0]),float(runtime._robot.get_joint_velocities()[0]));assert imeta['qualified'],imeta
  arr={}
  for j,p in enumerate(packets):
   arr.update({f'eye{j}_rgb':p['rgb'],f'eye{j}_depth_label':p['depth_label'],f'eye{j}_camera_usd_world_label':p['camera_usd_world_label'],f'eye{j}_view_raw':np.asarray(p['params']['cameraViewTransform']),f'eye{j}_projection_raw':np.asarray(p['params']['cameraProjection']),f'eye{j}_stage_usd_readback_label':get_cam_pose(stage,[LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH][j]),f'eye{j}_render_frame':p['render_frame'],f'eye{j}_render_time':p['render_time']})
  pos,q=runtime._robot.get_world_pose();arr.update(root_position_physics_label=pos,root_quaternion_physics_label=q,joints_current=runtime._robot.get_joint_positions(),source_time_s=clock[0],raw_time_s=clock[2]);fp=out/'frames'/f'frame_{i:03d}.npz';np.savez_compressed(fp,**arr);rows.append(dict(frame=fp.name,sha256=hashlib.sha256(fp.read_bytes()).hexdigest(),clock=clock,pair_error=pair_error,imu=imeta,render_frames=previous));print('qualified native exposure',i+1,'/8',flush=True)
 (out/'capture_seal.json').write_text(json.dumps(dict(complete=True,rows=rows,physics_motion_native=True,no_teleport=True,no_initialized_camera_pose_changes=True,no_model=True,no_navigation=True,renderer_pose_depth_offline_labels=True,IMU=dict(orientation_read=False,read_gravity=True,filter_sizes=[1,1,1],period_s=.02,native_sensor=True)),indent=2)+'\n');code=0
except BaseException:
 (out/'exception.txt').write_text(traceback.format_exc());raise
finally:
 def export():
  np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
  try:clock=runtime._clock_metadata()
  except RuntimeError:clock=None
  (out/'terminal.json').write_text(json.dumps(dict(exit_code=code,frames=len(rows),clock=clock,contacts=runtime.collision_contact_samples,no_navigation=True))+'\n')
 export_then_close(runtime,export,code)
