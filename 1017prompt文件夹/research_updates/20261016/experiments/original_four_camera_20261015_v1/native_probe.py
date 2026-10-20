from pathlib import Path
import sys,json,hashlib,numpy as np,traceback
D=Path(__file__).resolve().parent;W=D.parents[1];C=D.parent/'camera_exposure_contract_20261015_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(D),str(C),str(D.parent/'stereo_memory_navigation_20261004_v1'),str(P)]
from run_original_E1_direct_memory_navigation_v1 import IsaacBenchmarkRuntime,IsaacRuntimeConfig,DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,hide_diagnostic_markers,install_sigterm_cleanup,export_then_close
from benchmark.manifest import BenchmarkEpisode
from exposure_contract import attach_camera_parameters
from four_camera import NAMES,nominal_transforms,capture
from imu_interval_packet import interval_packet
from scipy.spatial.transform import Rotation
cfg=json.loads((D/'protocol.json').read_text());out=D/sys.argv[1];assert out.name in ['native_run','recovery_run'];out.mkdir(exist_ok=False);(out/'frames').mkdir();imu={};rear={};rows=[];code=1;transforms=nominal_transforms(.2,.3,.05)
for f,h in cfg['source_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h
class Runtime(IsaacBenchmarkRuntime):
 def _initialize(self,*args,**kwargs):
  import isaacsim
  Original=isaacsim.SimulationApp
  def own_app(*aa,**kk):
   app=Original(*aa,**kk);self._simulation_app=app;return app
  isaacsim.SimulationApp=own_app
  try:return super()._initialize(*args,**kwargs)
  finally:isaacsim.SimulationApp=Original
 @staticmethod
 def _create_markers(stage,ep,path,gf,usd_geom):
  result=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom);hide_diagnostic_markers(stage,usd_geom)
  import omni.kit.commands
  from omni.isaac.sensor import Camera
  from isaacsim.sensors.physics import _sensor
  from collect_data_dubins import BASE_ROOT,LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH
  from bev_ground_truth import enable_camera_depth
  # The existing front primitives are observed, never rewritten.
  for j,path in enumerate([LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH]):
   prim=stage.GetPrimAtPath(path);T=np.asarray(usd_geom.Xformable(prim).GetLocalTransformation()).T
   assert np.max(abs(T-transforms[j]))<2e-6,('Original front differs',path,T.tolist())
  for j,name in enumerate(NAMES[2:],2):
   path=BASE_ROOT+'/cam_'+name;assert not stage.GetPrimAtPath(path)
   prim=usd_geom.Camera.Define(stage,path);prim.GetFocalLengthAttr().Set(16.45);prim.GetHorizontalApertureAttr().Set(18.);prim.GetVerticalApertureAttr().Set(13.5);prim.GetClippingRangeAttr().Set(gf.Vec2f(.01,1000.))
   xf=usd_geom.Xformable(prim);xf.ClearXformOpOrder();xf.AddTransformOp().Set(gf.Matrix4d(*transforms[j].T.flatten().tolist()))
   cam=Camera(prim_path=path,resolution=(640,480),frequency=10)
   q=Rotation.from_matrix(transforms[j][:3,:3]).as_quat();cam.set_local_pose(translation=transforms[j][:3,3],orientation=np.r_[q[3],q[:3]],camera_axes='usd');cam.initialize();enable_camera_depth(cam);rear[name]=cam
  path=BASE_ROOT+'/original_four_native_imu';ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/original_four_native_imu',parent=BASE_ROOT,sensor_period=.02,translation=gf.Vec3d(0,0,0),orientation=gf.Quatd(1,0,0,0),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1);assert ok and prim is not None;imu.update(path=path,interface=_sensor.acquire_imu_sensor_interface());return result
runtime=Runtime(IsaacRuntimeConfig(scene_root=Path(cfg['scene_root']),contract_path=DEFAULT_CONTRACT.resolve(),occupancy_path=Path(cfg['occupancy']),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),gui=False,record_video=False,lighting_profile=cfg['lighting_profile'],collision_mode='physical_contact_v2',diagnostic_open_plane=False,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3))
install_sigterm_cleanup()
try:
 ep=BenchmarkEpisode.from_dict(next(e for e in json.loads(Path(cfg['episode_manifest']).read_text())['episodes'] if e['episode_id']=='golden_e1_0001'));runtime.reset(ep)
 import omni.usd
 from pxr import UsdGeom
 from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,BASE_ROOT,get_cam_pose
 cams=[runtime._left_camera,runtime._right_camera,rear['rear_left'],rear['rear_right']];paths=[LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,BASE_ROOT+'/cam_rear_left',BASE_ROOT+'/cam_rear_right']
 for c in cams:attach_camera_parameters(c);c.set_frequency(-1)
 stage=omni.usd.get_context().get_stage();stage.GetRootLayer().Export(str(out/'stage_root_initial.usda'))
 used=[]
 for l in stage.GetUsedLayers():
  p=Path(l.realPath) if l.realPath else None;used.append(dict(identifier=l.identifier,real_path=str(p) if p else None,sha256=hashlib.sha256(p.read_bytes()).hexdigest() if p and p.is_file() else hashlib.sha256(l.ExportToString().encode()).hexdigest()))
 (out/'stage_identity.json').write_text(json.dumps(dict(used_layers=used,meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)),lighting=cfg['lighting_profile'],lamp_positions=runtime.lighting_lamp_positions_world_m),indent=2)+'\n')
 def local_identity():
  rows=[]
  for j,path in enumerate(paths):
   prim=stage.GetPrimAtPath(path);cam=UsdGeom.Camera(prim);T=np.asarray(UsdGeom.Xformable(prim).GetLocalTransformation()).T;delta=float(np.max(abs(T-transforms[j])));assert delta<2e-6,(path,delta)
   values=[float(cam.GetFocalLengthAttr().Get()),float(cam.GetHorizontalApertureAttr().Get()),float(cam.GetVerticalApertureAttr().Get()),*map(float,cam.GetClippingRangeAttr().Get())];assert np.allclose(values,[16.45,18,13.5,.01,1000],atol=2e-6,rtol=0)
   rows.append(dict(name=NAMES[j],path=path,local_matrix=T.tolist(),nominal_error=delta,optics=values))
  return rows
 initial=local_identity();previous=[-1]*4
 for i in range(8):
  for unused in range(5):runtime.execute(np.zeros(10,np.float32))
  clock=runtime._clock_metadata();runtime._world.render();runtime._world.render();assert runtime._clock_metadata()==clock
  packets,err=capture(cams,previous,float(clock[2]),cfg['focal_px'],transforms);previous=[p['render_frame'] for p in packets];local=local_identity()
  raw=runtime.physics_diagnostics['raw_simulation_time_s'];reading=imu['interface'].get_sensor_reading(imu['path'],use_latest_data=True,read_gravity=True);_,imeta=interval_packet(reading,float(raw[-2]),float(raw[-1]),float(clock[0]),float(runtime._robot.get_joint_velocities()[0]));assert imeta['qualified'],imeta
  arr={}
  for j,(name,p) in enumerate(zip(NAMES,packets)):
   arr.update({name+'_rgb':p['rgb'],name+'_depth_label':p['depth_label'],name+'_camera_usd_world_label':p['camera_usd_world_label'],name+'_view_raw':np.asarray(p['params']['cameraViewTransform']),name+'_projection_raw':np.asarray(p['params']['cameraProjection']),name+'_stage_usd_readback_label':get_cam_pose(stage,paths[j]),name+'_render_frame':p['render_frame'],name+'_render_time':p['render_time']})
  pos,q=runtime._robot.get_world_pose();arr.update(root_position_physics_label=pos,root_quaternion_physics_label=q,joints_current=runtime._robot.get_joint_positions(),source_time_s=clock[0],raw_time_s=clock[2]);fp=out/'frames'/f'frame_{i:03d}.npz';np.savez_compressed(fp,**arr);rows.append(dict(frame=fp.name,sha256=hashlib.sha256(fp.read_bytes()).hexdigest(),clock=clock,pair_error=err,imu=imeta,render_frames=previous,local_identity=local));print('qualified four-camera exposure',i+1,'/8',flush=True)
 capture_result=dict(complete=True,camera_names=NAMES,rows=rows,initial_local_identity=initial,original_front_unchanged=True,rear_nominal_from_user_symmetry=True,physics_motion_native=True,no_teleport=True,no_initialized_camera_pose_changes=True,no_model=True,no_navigation=True,renderer_pose_depth_offline_labels=True,IMU=dict(orientation_read=False,read_gravity=True,filter_sizes=[1,1,1],period_s=.02,native_sensor=True));code=0
except BaseException:
 (out/'exception.txt').write_text(traceback.format_exc());raise
finally:
 def export():
  np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
  try:clock=runtime._clock_metadata()
  except RuntimeError:clock=None
  (out/'terminal.json').write_text(json.dumps(dict(exit_code=code,frames=len(rows),clock=clock,contacts=runtime.collision_contact_samples,no_navigation=True))+'\n')
 try:
  export()
  if code==0:(out/'capture_seal.json').write_text(json.dumps(capture_result,indent=2)+'\n')
 except BaseException:
  code=1
  (out/'exception.txt').write_text(traceback.format_exc())
 finally:runtime.close(return_code=code)
