"""ONE isolated free cube SDK timestamp/content witness, not robot/filter/navigation."""
from pathlib import Path
import sys,json,hashlib,datetime,traceback
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'imu_event_witness_protocol.json').read_text())
for f,s in p['source_sha256'].items():assert sha(f)==s,f
out=H/'imu_event_witness_run';out.mkdir(exist_ok=False)
import numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
rows=[];code=1
try:
 import omni.kit.commands
 import omni.usd
 from isaacsim.core.api import World
 from isaacsim.core.api.objects import DynamicCuboid
 from isaacsim.sensors.physics import _sensor
 from pxr import Gf,PhysxSchema
 from scipy.spatial.transform import Rotation
 world=World(stage_units_in_meters=1.,physics_dt=.02,rendering_dt=.1)
 world.get_physics_context().set_gravity(0.)
 cube=world.scene.add(DynamicCuboid(prim_path='/World/EventCube',name='event_cube',position=np.array([0.,0.,1.]),size=.1,mass=1.))
 api=PhysxSchema.PhysxRigidBodyAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath('/World/EventCube'));api.CreateAngularDampingAttr().Set(0.);api.CreateLinearDampingAttr().Set(0.)
 ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/imu',parent='/World/EventCube',sensor_period=.02,translation=Gf.Vec3d(0,0,0),orientation=Gf.Quatd(1,0,0,0),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1);assert ok and prim is not None
 interface=_sensor.acquire_imu_sensor_interface();world.reset();started=now()
 for i in range(50):
  command=p['rates_rad_s'][i%6];cube.set_angular_velocity(np.array([0.,0.,command]));before=float(world.current_time);previous=interface.get_sensor_reading('/World/EventCube/imu',use_latest_data=True,read_gravity=True)
  # Existing SDK step forwards exactly its current_time, and increments timer callback.
  world.step(render=False);after=float(world.current_time);reading=interface.get_sensor_reading('/World/EventCube/imu',use_latest_data=True,read_gravity=True)
  position,q=cube.get_world_pose();omega=cube.get_angular_velocity();R=Rotation.from_quat(np.array(q)[[1,2,3,0]]).as_matrix();native=np.array([reading.ang_vel_x,reading.ang_vel_y,reading.ang_vel_z]);body=R.T@omega
  rows.append(dict(step=i,begin_s=before,end_s=after,sensor_time_s=float(reading.time),valid=bool(reading.is_valid),native_z_rad_s=float(native[2]),native_gyro_body_rad_s=native.tolist(),current_rate_rad_s=float(command),actual_omega_world_rad_s=np.asarray(omega).tolist(),actual_quaternion_wxyz=np.asarray(q).tolist(),current_body_error_rad_s=float(np.linalg.norm(native-body)),previous_read_time_s=float(previous.time),previous_read_valid=bool(previous.is_valid),native_orientation_read=False))
 (out/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');(out/'identity.json').write_text(json.dumps(dict(started_utc=started,ended_utc=now(),steps=len(rows),physics_dt_s=.02,gravity=0.,damping=0.,mode='isolated free cube knownZ excitation',robot_runs=0,navigation_runs=0,predictor_calls=0,source_sha256=p['source_sha256']),indent=2)+'\n');code=0
except BaseException:
 (out/'failure.txt').write_text(traceback.format_exc());raise
finally:
 if not (out/'rows.json').exists():(out/'partial_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
 app.close()
print('EVENT_WITNESS_COMPLETE',len(rows),flush=True)
