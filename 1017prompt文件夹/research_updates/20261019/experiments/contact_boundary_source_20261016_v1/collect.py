"""Fresh four-camera data driven by frozen requests, with causal native IMU.

This is acquisition, not autonomous navigation or evaluation of old candidates.
Scene geometry/rendered poses/depth are labels only. Initial gauge and flat
height prior are disclosed privileges of the inherited IMU/encoder estimator.
"""
from pathlib import Path
import sys,json,hashlib,traceback
import numpy as np
from scipy.spatial.transform import Rotation

D=Path(__file__).resolve().parent;W=D.parents[1]
H=W/'experiments/stereo_memory_navigation_20261004_v1'
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
OLD=W/'experiments/user_fov_position_only_20261015_v1'
sys.path[:0]=list(map(str,[OLD,H,P,W/'experiments/camera_exposure_contract_20261015_v1']))
from run_original_E1_direct_memory_navigation_v1 import IsaacBenchmarkRuntime,IsaacRuntimeConfig,DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,hide_diagnostic_markers,install_sigterm_cleanup
from benchmark.manifest import BenchmarkEpisode
from exposure_contract import attach_camera_parameters
from four_camera import NAMES,nominal_transforms,capture
from imu_interval_packet import interval_packet
from imu_gravity_rolling import GravityRolling
sys.path.insert(0,str(W/'experiments/oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import install_runtime

sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    condition=sys.argv[1];cfg=json.loads((D/'protocol.json').read_text());spec=cfg['acquisitions'][condition]
    for p,h in cfg['source_sha256'].items():assert sha(p)==h,p
    for p,h in spec['input_sha256'].items():assert sha(p)==h,p
    root_name=sys.argv[2] if len(sys.argv)>2 else condition+'_run'
    assert root_name in [condition+'_run',condition+'_recovery_run']
    out=D/root_name;out.mkdir(exist_ok=False);(out/'frames').mkdir()
    (out/'protocol_at_dispatch.json').write_text(json.dumps(cfg,indent=2)+'\n')
    request=np.load(spec['request_program'])['requests'];assert request.shape==(spec['request_blocks'],10)
    transforms=nominal_transforms(.18197,.32506,.11503)
    imu={};cameras={};native=[];rows=[];gauge=None;estimator=None;code=1;termination='runtime_error'
    writers={};executed=[]
    class Runtime(IsaacBenchmarkRuntime):
        def _initialize(self,*args,**kwargs):
            import isaacsim
            Original=isaacsim.SimulationApp
            def own_app(*aa,**kk):
                app=Original(*aa,**kk);self._simulation_app=app;return app
            isaacsim.SimulationApp=own_app
            try:return super()._initialize(*args,**kwargs)
            finally:isaacsim.SimulationApp=Original
        def _configured_articulation_gains(self):
            gains=super()._configured_articulation_gains()
            if not hasattr(self,'_interval_original_step'):
                self._interval_original_step=self._world.step
                def measured_step(*args,**kwargs):
                    begin=float(self._world.current_time);r=self._interval_original_step(*args,**kwargs)
                    self._last_native_interval=(begin,float(self._world.current_time));return r
                self._world.step=measured_step
            return gains
        def _record_physics_diagnostic(self,*args,**kwargs):
            nonlocal gauge,estimator
            super()._record_physics_diagnostic(*args,**kwargs)
            if gauge is None:
                q=np.asarray(self._physics_diagnostics['root_quaternion_world_wxyz'][0])
                R0=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
                p0=np.asarray(self._physics_diagnostics['root_position_world_m'][0]).copy()
                gauge=dict(position=p0.tolist(),rotation=R0.tolist(),initial_only=True,privileged=True,
                    fixed_flat_base_height_m=.4000000059604645)
                estimator=GravityRolling(R0.T@np.array([0.,0.,1.]))
            reading=imu['interface'].get_sensor_reading(imu['path'],use_latest_data=True,read_gravity=True)
            begin,end=self._last_native_interval
            clock=self._clock_metadata()
            packet,meta=interval_packet(reading,begin,end,float(clock[0]),float(self._robot.get_joint_velocities()[0]))
            assert meta['qualified'],meta
            prediction=estimator.update(packet);R0=np.asarray(gauge['rotation'])
            position=np.asarray(gauge['position'])+R0@prediction['position'];position[2]=.4000000059604645
            T=np.eye(4);T[:3,:3]=R0@prediction['rotation'];T[:3,3]=position
            native.append(dict(**meta,main_encoder_rad_s=packet['main_encoder_rad_s'],estimated_base_world=T.tolist(),
                gravity_used=bool(prediction['gravity_used']),causal_before_next_action=True))
        @staticmethod
        def _create_markers(stage,ep,path,gf,usd_geom):
            result=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom)
            hide_diagnostic_markers(stage,usd_geom)
            import omni.kit.commands
            from omni.isaac.sensor import Camera
            from isaacsim.sensors.physics import _sensor
            from collect_data_dubins import BASE_ROOT
            from bev_ground_truth import enable_camera_depth
            for j,name in enumerate(NAMES):
                path=BASE_ROOT+'/contact_data_'+name;assert not stage.GetPrimAtPath(path)
                prim=usd_geom.Camera.Define(stage,path);prim.GetFocalLengthAttr().Set(cfg['focal_mm'])
                prim.GetHorizontalApertureAttr().Set(18.);prim.GetVerticalApertureAttr().Set(13.5)
                prim.GetClippingRangeAttr().Set(gf.Vec2f(.01,1000.))
                xf=usd_geom.Xformable(prim);xf.ClearXformOpOrder();xf.AddTransformOp().Set(gf.Matrix4d(*transforms[j].T.flatten().tolist()))
                cam=Camera(prim_path=path,resolution=(640,480),frequency=10)
                q=Rotation.from_matrix(transforms[j][:3,:3]).as_quat()
                cam.set_local_pose(translation=transforms[j][:3,3],orientation=np.r_[q[3],q[:3]],camera_axes='usd')
                cam.initialize();enable_camera_depth(cam);attach_camera_parameters(cam);cameras[name]=cam
            path=BASE_ROOT+'/contact_data_native_imu'
            ok,prim=omni.kit.commands.execute('IsaacSensorCreateImuSensor',path='/contact_data_native_imu',parent=BASE_ROOT,
                sensor_period=.02,translation=gf.Vec3d(0,0,0),orientation=gf.Quatd(1,0,0,0),
                linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1)
            assert ok and prim is not None
            imu.update(path=path,interface=_sensor.acquire_imu_sensor_interface());return result
    gains=json.loads((W/'experiments/oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json').read_text())['gains']
    Runtime=install_runtime(Runtime,**gains,output=out/'feedback')
    runtime=Runtime(IsaacRuntimeConfig(scene_root=Path(spec['scene_root']),contract_path=DEFAULT_CONTRACT.resolve(),
        occupancy_path=Path(spec['occupancy']),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),gui=False,
        record_video=False,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',
        diagnostic_open_plane=False,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3))
    install_sigterm_cleanup();previous=[-1]*4
    try:
        ep=BenchmarkEpisode.from_dict(next(e for e in json.loads(Path(spec['episode_manifest']).read_text())['episodes'] if e['episode_id']==spec['episode_id']))
        runtime.reset(ep)
        import omni.usd,cv2
        from pxr import UsdGeom
        from collect_data_dubins import BASE_ROOT,get_cam_pose
        stage=omni.usd.get_context().get_stage();cams=[cameras[n] for n in NAMES]
        for c in cams:c.set_frequency(-1)
        stage.GetRootLayer().Export(str(out/'stage_root_initial.usda'))
        layers=[]
        for layer in stage.GetUsedLayers():
            path=Path(layer.realPath) if layer.realPath else None
            layers.append(dict(identifier=layer.identifier,real_path=str(path) if path else None,
                sha256=sha(path) if path and path.is_file() else hashlib.sha256(layer.ExportToString().encode()).hexdigest()))
        (out/'stage_identity.json').write_text(json.dumps(dict(used_layers=layers,meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)),lighting=cfg['lighting_profile']),indent=2)+'\n')
        # Actual scene primitives are exported strictly as offline label metadata.
        cache=UsdGeom.XformCache();shapes=[];unsupported=[]
        for prim in stage.Traverse():
            if not str(prim.GetPath()).startswith('/World/Scene/'):continue
            if UsdGeom.Imageable(prim) and UsdGeom.Imageable(prim).ComputeVisibility()=='invisible':continue
            if prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Sphere):
                T=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                if not np.allclose(T[:3,:3],np.eye(3),atol=1e-6):
                    unsupported.append(str(prim.GetPath()));continue
                item=dict(path=str(prim.GetPath()),type=str(prim.GetTypeName()),center=T[:3,3].tolist(),
                    radius=float(prim.GetAttribute('radius').Get()))
                if prim.IsA(UsdGeom.Cylinder):
                    item.update(height=float(prim.GetAttribute('height').Get()),axis=str(UsdGeom.Cylinder(prim).GetAxisAttr().Get()))
                    if item['axis']!='Z':unsupported.append(str(prim.GetPath()));continue
                shapes.append(item)
        assert shapes,'No actual scene primitive metadata'
        (out/'scene_primitives.json').write_text(json.dumps(dict(objects=shapes,unsupported=unsupported,role='GT labels only; visible child meshes represented by their primitive parent when supported'),indent=2)+'\n')
        identity=[]
        for j,name in enumerate(NAMES):
            path=BASE_ROOT+'/contact_data_'+name;prim=stage.GetPrimAtPath(path);cam=UsdGeom.Camera(prim)
            T=np.asarray(UsdGeom.Xformable(prim).GetLocalTransformation()).T
            assert np.max(abs(T-transforms[j]))<2e-6
            optics=[float(cam.GetFocalLengthAttr().Get()),float(cam.GetHorizontalApertureAttr().Get()),float(cam.GetVerticalApertureAttr().Get()),*map(float,cam.GetClippingRangeAttr().Get())]
            assert np.allclose(optics,[cfg['focal_mm'],18,13.5,.01,1000],atol=2e-6,rtol=0)
            identity.append(dict(name=name,path=path,local_matrix=T.tolist(),optics=optics))
            wr=cv2.VideoWriter(str(out/(name+'.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),2.,(640,480));assert wr.isOpened();writers[name]=wr
        def save_exposure(block,terminal=False):
            nonlocal previous
            clock=runtime._clock_metadata()
            # Bound renderer priming without advancing the native physics/IMU clock.
            for unused in range(8):runtime._world.render()
            assert clock==runtime._clock_metadata()
            try:packets,error=capture(cams,previous,float(clock[2]),cfg['focal_px'],transforms)
            except Exception:
                diagnostic=[]
                for name,c in zip(NAMES,cams):
                    p=c.get_current_frame();params=p.get('research_camera_params')
                    diagnostic.append(dict(camera=name,render_frame=p.get('rendering_frame'),
                        render_time=p.get('rendering_time'),RGB_shape=list(np.asarray(p.get('rgba')).shape),
                        parameters={k:np.asarray(v).tolist() for k,v in params.items()} if isinstance(params,dict) else None))
                (out/'packet_failure_diagnostic.json').write_text(json.dumps(diagnostic,default=str,indent=2)+'\n')
                raise
            previous=[p['render_frame'] for p in packets]
            assert native and abs(native[-1]['time_s']-float(clock[0]))<1e-8
            body_est=np.asarray(native[-1]['estimated_base_world']);data={}
            for j,(name,p) in enumerate(zip(NAMES,packets)):
                data.update({name+'_rgb':p['rgb'],name+'_depth_label':p['depth_label'],
                    name+'_camera_usd_world_label':p['camera_usd_world_label'],
                    name+'_camera_usd_world_estimated':body_est@np.asarray(identity[j]['local_matrix']),
                    name+'_view_raw':np.asarray(p['params']['cameraViewTransform']),
                    name+'_projection_raw':np.asarray(p['params']['cameraProjection']),
                    name+'_stage_usd_readback_label':get_cam_pose(stage,identity[j]['path']),
                    name+'_render_frame':p['render_frame'],name+'_render_time':p['render_time']})
                writers[name].write(cv2.cvtColor(p['rgb'],cv2.COLOR_RGB2BGR))
            pos,q=runtime._robot.get_world_pose()
            data.update(root_position_physics_label=pos,root_quaternion_physics_label=q,
                joints_current=runtime._robot.get_joint_positions(),source_time_s=clock[0],raw_time_s=clock[2],
                native_packet_index=len(native)-1)
            path=out/'frames'/f'frame_{len(rows):03d}.npz';np.savez_compressed(path,**data)
            rows.append(dict(frame=path.name,sha256=sha(path),clock=clock,pair_error=error,
                imu_index=len(native)-1,render_frames=previous,request_blocks_executed=block,terminal=terminal))
            (out/'progress.json').write_text(json.dumps(dict(frames=len(rows),blocks=block,time_s=clock[0]))+'\n')
            if len(rows)%20==1:print(condition,'four-camera frames',len(rows),'time',clock[0],flush=True)
        save_exposure(0)
        for i,action in enumerate(request):
            current=runtime.execute(action);executed.append(action.tolist())
            collision=bool(current.collision);last=i+1==len(request)
            if (i+1)%5==0 or collision or last:save_exposure(i+1,collision or last)
            if collision:termination='physical_contact';break
        else:termination='request_program_complete'
        for wr in writers.values():wr.release()
        (out/'initial_gauge.json').write_text(json.dumps(gauge,indent=2)+'\n')
        (out/'native_imu.json').write_text(json.dumps(native,separators=(',',':'))+'\n')
        np.savez_compressed(out/'requests.npz',requests=np.asarray(executed,np.float32))
        np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
        (out/'contacts.json').write_text(json.dumps(runtime.collision_contact_samples)+'\n')
        (out/'terminal.json').write_text(json.dumps(dict(exit_code=0,termination=termination,
            requested_blocks=len(request),executed_blocks=len(executed),frames=len(rows),
            time_s=runtime._clock_metadata()[0],is_navigation=False,no_GT_online=True,
            lowlevel_bank_feedback_ideal_IMU_privileged=True,closed_loop_source=None))+'\n')
        (out/'capture_seal.json').write_text(json.dumps(dict(complete=True,camera_names=NAMES,rows=rows,
            initial_local_identity=identity,native_imu_packets=len(native),condition=condition,role=spec['role'],
            renderer_pose_depth_offline_labels=True,initial_gauge_privileged=True,flat_height_prior_m=.4000000059604645,
            no_model=True,no_navigation=True,no_camera_mutation_after_initialize=True,
            original_project_modified=False,request_program_sha256=sha(spec['request_program'])),indent=2)+'\n')
        code=0;print('COMPLETE',condition,termination,len(rows),flush=True)
    except BaseException:
        (out/'exception.txt').write_text(traceback.format_exc());raise
    finally:
        for wr in writers.values():wr.release()
        if code:
            np.savez_compressed(out/'physics_partial.npz',**runtime.physics_diagnostics)
            (out/'native_imu_partial.json').write_text(json.dumps(native,separators=(',',':'))+'\n')
            (out/'terminal_partial.json').write_text(json.dumps(dict(exit_code=code,frames=len(rows),blocks=len(executed)))+'\n')
        runtime.close(return_code=code)

if __name__=='__main__':main()
