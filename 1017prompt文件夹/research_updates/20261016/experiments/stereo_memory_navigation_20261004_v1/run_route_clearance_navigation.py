"""Shared map/visible information diagnostic; only the permitted current packet influences requests."""
from dataclasses import replace
import hashlib,json,sys,os
from pathlib import Path
import numpy as np
import scripts.run_pointgoal_benchmark as runner

HERE=Path(__file__).parent
BASELINE=HERE.parent/'visible_surface_connectivity_20261004_v1'
from render_flush import fixed_render_flush
from capture_transaction import replace_step_images
from route_clearance_policy import InformationPolicy
from execution_snapshot import export_execution
from rig_inputs import validate_capture,validate_mask_identity
from surface_support import native_ground_mesh
from scripts.camera_rig import camera_rig_metadata
BOUNDARY=HERE.parent/'bilateral_temporal_geometry_navigation_20261003_v1'
sys.path.insert(0,str(BOUNDARY))
from render_boundary import hide_diagnostic_markers,marker_visibility
sys.path.insert(0,str(HERE.parent/'clean_visual_hard0_intervention_20261003_v1'))
from clock_sync import ensure_current_frames
ROLL=HERE.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'
sys.path.insert(0,str(ROLL))
from roll_pid import install_runtime

args=runner._build_parser().parse_args()
MODE=os.environ['INFORMATION_MODE'];assert MODE=='visible'
MEMORY_MODE=os.environ['MEMORY_MODE'];assert MEMORY_MODE=='history'
REPRESENTATION=os.environ.get('MAP_REPRESENTATION','points');assert REPRESENTATION=='points'
ACQUISITION='every_render'
assert ACQUISITION in ('original','every_render')
config=json.loads((ROLL/'selected_candidate.json').read_text())
base=install_runtime(runner.IsaacBenchmarkRuntime,**config['gains'],output=args.run_root/'feedback')
instances=[];CURRENT_PACKET=None
validate_mask_identity(json.loads((BASELINE/'mask_identity.json').read_text()))
SELF_MASK=np.load(BASELINE/'base_link_masks.npz')
y,x=np.mgrid[0:480:4,0:640:4];FOCAL=16.45/18*640
RAYS=np.column_stack(((x.ravel()-319.5)/FOCAL,(y.ravel()-239.5)/FOCAL,np.ones(x.size)))


def factory(name,protocol,seed,**kwargs):
    assert name=='goal_greedy'
    policy=InformationPolicy(protocol,MODE,REPRESENTATION,memory_mode=MEMORY_MODE)
    instances.append(policy);act=policy.act
    def record(observation,goal):
        assert CURRENT_PACKET is not None
        packet=dict(CURRENT_PACKET)
        if policy.belief is not None:packet.pop('known_grid',None)
        policy.set_current_packet(packet)
        action=act(observation,goal);i=len(policy.records)-1
        sensors=args.run_root/'privileged_sensor_packets';sensors.mkdir(exist_ok=True)
        np.savez_compressed(sensors/f'frame_{i:03d}.npz',points_world=packet['points_world'],root_xyz=packet['root_xyz'],time_s=packet['time_s'])
        if i%10==0:
            b=policy.belief;path=args.run_root/'belief_snapshots';path.mkdir(exist_ok=True)
            np.savez_compressed(path/f'frame_{i:03d}.npz',labels=b.labels,origin=b.origin,resolution=b.resolution,provenance=b.provenance(),floor_time=b.memory.floor_time,hazard_time=b.memory.hazard_time,footprint_geometry_mask=b.footprint_mask,time_s=b.time)
        dest=args.run_root/'online_inputs';dest.mkdir(exist_ok=True)
        np.savez_compressed(dest/f'frame_{i:03d}.npz',left_rgb=observation.stereo_left_rgb,right_rgb=observation.stereo_right_rgb,
            position_world_xy=observation.position_world_xy,yaw_world_rad=observation.yaw_world_rad,
            quaternion_world_wxyz_history=observation.quaternion_world_wxyz_history,
            proprio_history=observation.proprio_history,action_history=observation.action_history,
            pointgoal_base_xy=np.asarray(goal),request=action)
        return action
    policy.act=record;return policy


class RecordRuntime(base):
    @staticmethod
    def _create_markers(stage,episode,path,gf,usd_geom):
        actual=base._create_markers(stage,episode,path,gf,usd_geom)
        snapshot=hide_diagnostic_markers(stage,usd_geom)
        (args.run_root/'marker_visibility_at_creation.json').write_text(json.dumps(snapshot,indent=2)+'\n')
        return actual

    def __init__(self,config):
        super().__init__(replace(config,diagnostic_physics_trace=True));self._capture_index=0;self._marker_checks=[];self._camera_clocks=[];self._acquisition_enabled=False;self._execution_states=[]

    def _capture(self):
        global CURRENT_PACKET
        import omni.usd
        from pxr import UsdGeom
        from bev_ground_truth import read_camera_depth
        from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose,_camera_optical_to_base,_get_rgb
        stage=omni.usd.get_context().get_stage()
        if ACQUISITION=='every_render':
            if not self._acquisition_enabled:
                for camera in (self._left_camera,self._right_camera,self._third_camera):camera.set_frequency(-1)
                self._acquisition_enabled=True
            before_clock=self._clock_metadata();before_state=[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            ensure_current_frames((self._left_camera,self._right_camera),float(before_clock[2]),self._world.render)
            fixed_render_flush(self._world.render,self._clock_metadata,lambda:[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())])
            after_state=[np.asarray(v) for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            assert self._clock_metadata()==before_clock,'render advanced physics clock'
            assert all(np.array_equal(a,b) for a,b in zip(before_state,after_state)),'render changed physical state'
            if not (args.run_root/'initial_render_invariants.json').exists():(args.run_root/'initial_render_invariants.json').write_text(json.dumps({'clock_unchanged':True,'physical_state_unchanged':True})+'\n')
        visibility=marker_visibility(stage,UsdGeom)
        assert all(x['computed_visibility']=='invisible' for x in visibility.values())
        self._marker_checks.append(visibility);step=super()._capture()
        # The single base capture writes USD diagnostic-curve attributes.
        # Re-render after that transaction; do not capture/history-update twice.
        fixed_render_flush(self._world.render,self._clock_metadata,lambda:[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())])
        from collect_data_dubins import _get_rgb
        step=replace_step_images(step,_get_rgb(self._left_camera),_get_rgb(self._right_camera),_get_rgb(self._third_camera))
        if self.config.record_video:
            self._captured_frames['stereo_view'][-1]=step.frames['stereo_view'].copy()
            self._captured_frames['third_person'][-1]=step.frames['third_person'].copy()
        raw_time=float(step.raw_simulation_time_s)
        clock={'capture_index':self._capture_index,'raw_physics_time_s':raw_time,'episode_time_s':float(step.simulation_time_s),'eyes':{}}
        for eye,camera in (('left',self._left_camera),('right',self._right_camera)):
            frame=camera.get_current_frame();clock['eyes'][eye]={'rendering_time_s':float(frame['rendering_time']),'rendering_frame':int(frame['rendering_frame']),'configured_frequency':float(camera.get_frequency()),'age_s':raw_time-float(frame['rendering_time'])}
        self._camera_clocks.append(clock)
        if ACQUISITION=='every_render':
            assert all(abs(v['age_s'])<=1e-5 for v in clock['eyes'].values()),clock
            if len(self._camera_clocks)>1:
                prev=self._camera_clocks[-2]
                assert all(clock['eyes'][eye]['rendering_frame']>prev['eyes'][eye]['rendering_frame'] and clock['eyes'][eye]['rendering_time_s']>prev['eyes'][eye]['rendering_time_s'] for eye in ('left','right')),clock
        i=self._capture_index;self._capture_index+=1
        self._execution_states.append(export_execution(self._action_smoother,self._action_protocol) if hasattr(self._action_smoother,'pid') else None)
        position,q=self._robot.get_world_pose();dest=args.run_root/'offline_truth';dest.mkdir(exist_ok=True)
        validate_capture(dict(left_optical_to_base=_camera_optical_to_base(stage,LEFT_CAMERA_PATH),right_optical_to_base=_camera_optical_to_base(stage,RIGHT_CAMERA_PATH),left_rgb=_get_rgb(self._left_camera),right_rgb=_get_rgb(self._right_camera),raw_time_s=raw_time,left_render_time_s=clock['eyes']['left']['rendering_time_s'],right_render_time_s=clock['eyes']['right']['rendering_time_s']))
        np.savez_compressed(dest/f'frame_{i:03d}.npz',left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),
            left_camera_usd_world=get_cam_pose(stage,LEFT_CAMERA_PATH),right_camera_usd_world=get_cam_pose(stage,RIGHT_CAMERA_PATH),
            root_position_world_m=position,root_quaternion_world_wxyz=q,simulation_time_s=step.simulation_time_s)
        points=[];meshes=[]
        if MODE=='visible':
            for eye,camera,path in [('left',self._left_camera,LEFT_CAMERA_PATH),('right',self._right_camera,RIGHT_CAMERA_PATH)]:
                depth=read_camera_depth(camera)[::4,::4].ravel();pose=get_cam_pose(stage,path)
                rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
                valid=np.isfinite(depth)&(depth>=.45)&(depth<=8.)&(~SELF_MASK[eye][::4,::4].ravel())
                points.append(pose[:3,3]+(RAYS[valid]*depth[valid,None])@rotation.T)
                if REPRESENTATION=='surface':
                    meshes.append(native_ground_mesh(read_camera_depth(camera),pose,SELF_MASK[eye],np.asarray(position)))
        CURRENT_PACKET=dict(time_s=float(step.simulation_time_s),points_world=np.concatenate(points) if points else np.empty((0,3)),root_xyz=np.asarray(position),speed_xy=float(np.linalg.norm(self._robot.get_linear_velocity()[:2])))
        if REPRESENTATION=='surface':CURRENT_PACKET['ground_meshes']=meshes
        if MODE=='map':
            occ=self.occupancy;CURRENT_PACKET['known_grid']=dict(labels=np.asarray(occ.grid,dtype=np.int8),origin=np.asarray(occ.global_origin_xy),resolution=float(occ.resolution_m))
            if not (args.run_root/'truth_map.npz').exists():np.savez_compressed(args.run_root/'truth_map.npz',**CURRENT_PACKET['known_grid'])
        # Offline scoring label only; it is never inserted into CURRENT_PACKET.
        if not (args.run_root/'offline_occupancy_label.npz').exists():
            occ=self.occupancy
            np.savez_compressed(args.run_root/'offline_occupancy_label.npz',labels=np.asarray(occ.grid,dtype=np.int8),origin=np.asarray(occ.global_origin_xy),resolution=float(occ.resolution_m))
        return step

    def close(self,*a,**kw):
        if hasattr(self,'_robot'):
            np.savez_compressed(args.run_root/'physics.npz',**self.physics_diagnostics)
            def convert(value):
                if isinstance(value,np.ndarray):return value.tolist()
                raise TypeError(type(value))
            (args.run_root/'runtime_configuration.json').write_text(json.dumps(self.diagnostic_configuration,default=convert,indent=2)+'\n')
            (args.run_root/'execution_states.json').write_text(json.dumps(self._execution_states,indent=2)+'\n')
        if hasattr(self,'_camera_clocks'):(args.run_root/'camera_clocks.json').write_text(json.dumps(self._camera_clocks,indent=2)+'\n')
        if hasattr(self,'_marker_checks'):(args.run_root/'marker_visibility_each_capture.json').write_text(json.dumps(self._marker_checks)+'\n')
        if instances:
            policy=instances[-1]
            if policy.belief is not None:np.savez_compressed(args.run_root/'belief_final.npz',labels=policy.belief.labels,origin=policy.belief.origin,resolution=policy.belief.resolution)
            (args.run_root/'vision_records.json').write_text(json.dumps(policy.records,indent=2)+'\n')
            (args.run_root/'policy_states.json').write_text(json.dumps(policy.common_states,indent=2)+'\n')
            if policy._path_xy is not None:np.savez(args.run_root/'vision_reference.npz',position_world_xy=policy._path_xy)
        return super().close(*a,**kw)


runner._make_policy=factory;runner.IsaacBenchmarkRuntime=RecordRuntime
child_base=runner._child_command
def child(*a,**kw):
    command=child_base(*a,**kw);command[1]=str(Path(__file__).resolve());return command
runner._child_command=child
PROJECT=Path(runner.__file__).resolve().parents[1]
args.run_root.mkdir(parents=True,exist_ok=True)
sources=[HERE/'route_clearance_consumer.py',HERE/'route_clearance_policy.py',HERE/'route_clearance_navigation_protocol.json',HERE/'online_bank_hold.py',HERE/'online_bank_hold_policy.py',HERE/'online_bank_hold_navigation_protocol.json',HERE/'execution_snapshot.py',HERE/'run_stopping_navigation.py',HERE/'stopping_consumer.py',HERE/'stopping_policy.py',HERE/'primitive_planner.py',HERE/'local_stopping_protocol.json',HERE/'provenance_policy.py',HERE/'provenance_grid.py',HERE/'spatial_memory.py',HERE/'online_provenance_protocol.json',HERE/'capture_transaction.py',HERE/'render_flush.py',HERE/'transaction_navigation_protocol.json',HERE/'risk_policy.py',HERE/'risk_grid.py',HERE/'risk_protocol.json',BASELINE/'information_policy.py',BASELINE/'local_map.py',BASELINE/'surface_support.py',BASELINE/'rig_inputs.py',BASELINE/'base_link_masks.npz',BASELINE/'mask_identity.json',BOUNDARY/'render_boundary.py',ROLL/'roll_pid.py',Path(__file__),Path(runner.__file__).parent/'camera_rig.py',HERE.parent/'clean_visual_hard0_intervention_20261003_v1/clock_sync.py',Path(runner.__file__)]
sources.extend([PROJECT/'benchmark/policies.py',PROJECT/'benchmark/isaac_runtime.py',PROJECT/'benchmark/lighting.py',PROJECT/'training/action_protocol.py',PROJECT/'scripts/action_smoothing.py',PROJECT/'configs/data_contract.yaml',ROLL/'selected_candidate.json'])
(args.run_root/'experiment_identity.json').write_text(json.dumps(dict(actual_policy='route_clearance_policy.InformationPolicy',route_clearance_intervention='event-latched grid point clearance matches unchanged stop disc; .6 before event; no PID/cap/map changes',bank_source_intervention='online motion-to-zero-main event; hold previously emitted raw bank; unchanged parent resumes motion',local_stopping_consumer=True,certified_stopping_bound=False,memory_mode=MEMORY_MODE,live_provenance_consumer=True,body_prior_observed_free=False,unknown_labels_unchanged=True,information_mode=MODE,
    privileged=MODE!='rgb',diagnostic_only=True,shared_controller=False,shared_roll_pid_and_geometric_tracker=True,explicit_unknown_exploration=True,map_representation=REPRESENTATION,acquisition_mode='fixed3render_then_single_base_capture_then_fixed3render_transaction',camera_time_assertions=True,
    benchmark_location_attitude=True,renderer_depth_online=MODE=='visible',full_map_online=MODE=='map',oracle_reference_online=False,
    markers_hidden_and_checked_each_capture=True,camera_rig=camera_rig_metadata(),self_mask_sha256=hashlib.sha256((BASELINE/'base_link_masks.npz').read_bytes()).hexdigest(),lighting='fixed_camera_fill_v1',formal_budget_s=35,
    roll_pid_gains=config['gains'],source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}),indent=2)+'\n')
raise SystemExit(runner.main())
