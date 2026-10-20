"""One frozen interface pilot. No launch without exact-source reviewed admission."""
from pathlib import Path
import json,hashlib,sys,argparse,os,datetime
ROOT=Path(__file__).resolve().parents[2]
H=ROOT/'experiments/stereo_memory_navigation_20261004_v1'
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
def validate_admission(path):
 d=json.loads(Path(path).read_text());source=str(Path(__file__).resolve())
 if d.get('launch_admitted') is not True or d.get('physical_episodes_max')!=1 or d.get('retries')!=0 or source not in d.get('source_sha256',{}):raise ValueError('reviewed single-episode exact-source admission required')
 if any(hashlib.sha256(Path(f).read_bytes()).hexdigest()!=sha for f,sha in d['source_sha256'].items()):raise ValueError('source identity changed')
 return d

def reserve_attempt(admission_path):
 path=Path(str(Path(admission_path).resolve())+".attempt.json")
 with path.open("x") as f:
  json.dump(dict(status="reserved",pid=os.getpid(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),no_retry=True),f);f.flush();os.fsync(f.fileno())
 directory=os.open(path.parent,os.O_RDONLY)
 try:os.fsync(directory)
 finally:os.close(directory)
 return path

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--admission',required=True);parser.add_argument('--out',required=True);args=parser.parse_args();admission=validate_admission(args.admission)
 out=Path(args.out).resolve()
 if not out.is_relative_to(ROOT/'experiments'):raise ValueError('output must remain in workspace experiments')
 if out.exists():raise FileExistsError(out)
 reserve_attempt(args.admission)
 out.mkdir(exist_ok=False)
 for directory in (P,H,ROOT/'experiments/tube_bridge_20261012_v1',ROOT/'experiments/tube_anchor_provider_20261012_v1',ROOT/'experiments/oracle_feasible_route_20260929_v9/hybrid'):sys.path.insert(0,str(directory))
 import numpy as np
 from scipy.spatial.transform import Rotation
 import scripts.run_pointgoal_benchmark as runner
 from native_rolling_shadow import NativeRollingShadow
 from flat_ground_pose_bridge import FlatGroundPoseBridge
 from imu_interval_packet import interval_packet
 from render_boundary import hide_diagnostic_markers
 from complete_capture import complete_capture
 from render_flush import fixed_render_flush
 from clock_sync import ensure_current_frames
 from roll_pid import install_runtime
 from block_bridge import BlockBridge
 from provider import provide_clocked
 from probe_cleanup import install_sigterm_cleanup,seal_existing
 install_sigterm_cleanup()
 rolling=NativeRollingShadow();pose_bridge=None;native_interface=None;native_path=None;sensor_records=[];clock_records=[];windows=[]
 class CaptureRuntime(runner.IsaacBenchmarkRuntime):
  def _configured_articulation_gains(self):
   gains=super()._configured_articulation_gains()
   if not hasattr(self,'_interval_original_step'):
    self._interval_original_step=self._world.step
    def measured_step(*a,**kw):
     begin=float(self._world.current_time);result=self._interval_original_step(*a,**kw);self._actual_last_interval=(begin,float(self._world.current_time));return result
    self._world.step=measured_step
   return gains
  @staticmethod
  def _create_markers(stage,ep,path,gf,usd_geom):
   nonlocal native_interface,native_path
   result=runner.IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom);hide_diagnostic_markers(stage,usd_geom)
   import omni.kit.commands
   from isaacsim.sensors.physics import _sensor
   from collect_data_dubins import BASE_ROOT
   native_path=BASE_ROOT+'/research_native_imu'
   ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/research_native_imu',parent=BASE_ROOT,sensor_period=.02,translation=gf.Vec3d(0,0,0),orientation=gf.Quatd(1,0,0,0),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1)
   if not ok or prim is None:raise RuntimeError('native IMU creation failed')
   native_interface=_sensor.acquire_imu_sensor_interface();return result
  def _record_physics_diagnostic(self,*a,**kw):
   super()._record_physics_diagnostic(*a,**kw)
   reading=native_interface.get_sensor_reading(native_path,use_latest_data=True,read_gravity=True);begin,end=self._actual_last_interval
   if abs(end-self._physics_diagnostics['raw_simulation_time_s'][-1])>1e-10:raise RuntimeError('native interval mismatch')
   sensor,diag=interval_packet(reading,begin,end,self._physics_diagnostics['simulation_time_s'][-1],self._physics_diagnostics['joint_velocity_rad_s'][-1][0]);sensor_records.append(diag)
   if sensor is None:raise RuntimeError('native qualification failed; no fallback')
   normal=None
   if rolling.model is None:
    q=np.asarray(self._physics_diagnostics['root_quaternion_world_wxyz'][0]);normal=Rotation.from_quat(q[[1,2,3,0]]).as_matrix().T@np.array([0.,0.,1.])
   rolling.consume(sensor,normal)
   if pose_bridge is not None:pose_bridge.consume(rolling.records[-1])
  def _capture(self):
   nonlocal pose_bridge
   from collect_data_dubins import _get_rgb
   def snapshot():return [np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
   if not getattr(self,'fresh_cameras',False):
    for camera in (self._left_camera,self._right_camera,self._third_camera):camera.set_frequency(-1)
    self.fresh_cameras=True
   clock=self._clock_metadata();ensure_current_frames((self._left_camera,self._right_camera),clock[2],self._world.render)
   fixed_render_flush(self._world.render,self._clock_metadata,snapshot)
   step=complete_capture(super()._capture,self._world.render,self._clock_metadata,snapshot,lambda:(_get_rgb(self._left_camera),_get_rgb(self._right_camera),_get_rgb(self._third_camera)))
   self._captured_frames['stereo_view'][-1]=step.frames['stereo_view'].copy();self._captured_frames['third_person'][-1]=step.frames['third_person'].copy()
   if pose_bridge is None:
    p,q=self._robot.get_world_pose();R=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix();pose_bridge=FlatGroundPoseBridge(p,R)
    for record in rolling.records:pose_bridge.consume(record)
    (out/'initial_gauge.json').write_text(json.dumps(dict(position=np.asarray(p).tolist(),rotation=R.tolist(),privileged_initial_only=True,flat_ground_prior=True)))
   if self._clock_metadata()!=clock:raise RuntimeError('capture advanced physics')
   clock_records.append(list(clock));return step
 gains=json.loads((ROOT/'experiments/oracle_feasible_route_20260929_v9/hybrid/selected_candidate.json').read_text())['gains'];Runtime=install_runtime(CaptureRuntime,**gains,output=out/'feedback')
 reference=json.loads((H/'native_pose_baseline35_e0_protocol_v1.json').read_text());runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=Path(reference['scene_root']),contract_path=P/'configs/data_contract.yaml',occupancy_path=Path(reference['occupancy']),record_video=True,gui=False,diagnostic_physics_trace=True,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',auxiliary_velocity_limit_rad_s=.3))
 code=1;active=None
 try:
  episode=next(e for e in runner.load_manifest(reference['episode_manifest']).episodes if e.episode_id==reference['episode_id']);runtime.reset(episode)
  from collect_data_dubins import _get_rgb
  for index in range(70):
   active=BlockBridge(out/f'window_{index:03d}.jsonl',episode.episode_id);provide_clocked(active,pose_bridge,runtime,_get_rgb)
   initial=np.asarray(runtime._robot.get_world_pose()[0])[:2];active.commit(tuple((0.,0.) for _ in range(25)),initial_supervision_xy=tuple(float(v) for v in initial))
   for _ in range(5):
    active.block(runtime)
    if active.contact:break
   result=active.qualify();windows.append(dict(window=index,**result))
   if not result['scorable']:break
  code=0
 finally:
  if active is not None and not active.journal.closed:active.journal.close()
  (out/'windows.json').write_text(json.dumps(windows,indent=2));(out/'native_reads.json').write_text(json.dumps(sensor_records));(out/'rolling_records.json').write_text(json.dumps(rolling.records));(out/'capture_clocks.json').write_text(json.dumps(clock_records))
  np.savez_compressed(out/'physics.npz',**{k:np.asarray(v) for k,v in runtime._physics_diagnostics.items()})
  runtime.close(return_code=code);seal_existing(out,dict(process_return_code=code,interface_pilot_only=True,new_learning_training=False))
if __name__=='__main__':main()
