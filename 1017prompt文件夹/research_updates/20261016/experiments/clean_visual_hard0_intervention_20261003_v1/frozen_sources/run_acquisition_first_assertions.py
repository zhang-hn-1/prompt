"""Unmodified historical left-RGB controller, verified clean rendering."""
from dataclasses import replace
import hashlib,json,sys,os
from pathlib import Path
import numpy as np
import scripts.run_pointgoal_benchmark as runner

HERE=Path(__file__).parent
LEGACY=HERE.parent/'fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1'
sys.path.insert(0,str(LEGACY))
from policy_continuous import ContinuousVisionDetourPolicy
from policy import MODEL
BOUNDARY=HERE.parent/'bilateral_temporal_geometry_navigation_20261003_v1'
sys.path.insert(0,str(BOUNDARY))
from render_boundary import hide_diagnostic_markers,marker_visibility
ROLL=HERE.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'
sys.path.insert(0,str(ROLL))
from roll_pid import install_runtime

args=runner._build_parser().parse_args()
assert args.episode_id in ('hard_0_e1_0003','hard_0_e1_0004','golden_e0_0001','golden_e1_0001','golden_e2_0001')
ACQUISITION=os.environ.get('VISION_ACQUISITION_MODE','original')
assert ACQUISITION in ('original','every_render')
config=json.loads((ROLL/'selected_candidate.json').read_text())
base=install_runtime(runner.IsaacBenchmarkRuntime,**config['gains'],output=args.run_root/'feedback')
original_factory=runner._make_policy;instances=[]


def factory(name,protocol,seed,**kwargs):
    policy=ContinuousVisionDetourPolicy(protocol) if name=='goal_greedy' else original_factory(name,protocol,seed,**kwargs)
    instances.append(policy);act=policy.act
    def record(observation,goal):
        action=act(observation,goal);i=len(policy.records)-1
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
        super().__init__(replace(config,diagnostic_physics_trace=True));self._capture_index=0;self._marker_checks=[];self._camera_clocks=[];self._acquisition_enabled=False

    def _capture(self):
        import omni.usd
        from pxr import UsdGeom
        from bev_ground_truth import read_camera_depth
        from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose
        stage=omni.usd.get_context().get_stage()
        if ACQUISITION=='every_render' and not self._acquisition_enabled:
            before_clock=self._clock_metadata();before_state=[np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            for camera in (self._left_camera,self._right_camera,self._third_camera):camera.set_frequency(-1)
            self._world.render()
            after_state=[np.asarray(v) for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
            assert self._clock_metadata()==before_clock,'render advanced physics clock'
            assert all(np.array_equal(a,b) for a,b in zip(before_state,after_state)),'render changed physical state'
            (args.run_root/'initial_render_invariants.json').write_text(json.dumps({'clock_unchanged':True,'physical_state_unchanged':True})+'\n')
            self._acquisition_enabled=True
        visibility=marker_visibility(stage,UsdGeom)
        assert all(x['computed_visibility']=='invisible' for x in visibility.values())
        self._marker_checks.append(visibility);step=super()._capture()
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
        position,q=self._robot.get_world_pose();dest=args.run_root/'offline_truth';dest.mkdir(exist_ok=True)
        np.savez_compressed(dest/f'frame_{i:03d}.npz',left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),
            left_camera_usd_world=get_cam_pose(stage,LEFT_CAMERA_PATH),right_camera_usd_world=get_cam_pose(stage,RIGHT_CAMERA_PATH),
            root_position_world_m=position,root_quaternion_world_wxyz=q,simulation_time_s=step.simulation_time_s)
        return step

    def close(self,*a,**kw):
        if hasattr(self,'_robot'):np.savez_compressed(args.run_root/'physics.npz',**self.physics_diagnostics)
        if hasattr(self,'_camera_clocks'):(args.run_root/'camera_clocks.json').write_text(json.dumps(self._camera_clocks,indent=2)+'\n')
        if hasattr(self,'_marker_checks'):(args.run_root/'marker_visibility_each_capture.json').write_text(json.dumps(self._marker_checks)+'\n')
        if instances:
            policy=instances[-1];(args.run_root/'vision_records.json').write_text(json.dumps(policy.records,indent=2)+'\n')
            if policy._path_xy is not None:np.savez(args.run_root/'vision_reference.npz',position_world_xy=policy._path_xy)
        return super().close(*a,**kw)


runner._make_policy=factory;runner.IsaacBenchmarkRuntime=RecordRuntime
child_base=runner._child_command
def child(*a,**kw):
    command=child_base(*a,**kw);command[1]=str(Path(__file__).resolve());return command
runner._child_command=child
args.run_root.mkdir(parents=True,exist_ok=True)
sources=[LEGACY/'policy.py',LEGACY/'policy_continuous.py',LEGACY.parent/'pilot.py',BOUNDARY/'render_boundary.py',ROLL/'roll_pid.py',Path(__file__),HERE/'PROTOCOL.md',HERE/'ACQUISITION_AUDIT.md']
(args.run_root/'experiment_identity.json').write_text(json.dumps(dict(actual_policy=ContinuousVisionDetourPolicy.policy_name,
    privileged=False,legacy_control_unchanged=True,acquisition_mode=ACQUISITION,camera_time_assertions=ACQUISITION=='every_render',uses_left_rgb_for_control=True,right_rgb_record_only=True,
    benchmark_location_attitude=True,renderer_depth_online=False,oracle_reference_online=False,
    markers_hidden_and_checked_each_capture=True,camera_rig='frozen',lighting='fixed_camera_fill_v1',formal_budget_s=35,
    roll_pid_gains=config['gains'],model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
    source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}),indent=2)+'\n')
raise SystemExit(runner.main())
