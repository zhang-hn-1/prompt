"""Actual online map-free RGB policy; depth saved only by runtime for scoring."""
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import sys
import cv2
import numpy as np
import scripts.run_pointgoal_benchmark as runner
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from policy import BilateralTemporalPolicy
from render_boundary import hide_diagnostic_markers, marker_visibility

WORKER=Path(__file__).resolve()
HERE=WORKER.parent.parent
ROLL=HERE.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'
sys.path.insert(0,str(ROLL))
from roll_pid import install_runtime

cv2.setNumThreads(2)
args=runner._build_parser().parse_args()
mode=os.environ['BILATERAL_TEMPORAL_MODE']
if mode not in ('b0','candidate') or args.episode_id not in ('golden_e1_0001','golden_e2_0001'):
    raise ValueError('outside frozen four-run development probe')
if not (HERE/'offline_assessment.json').exists():raise ValueError('offline assessment must precede online probe')
config=json.loads((ROLL/'selected_candidate.json').read_text())
base=install_runtime(runner.IsaacBenchmarkRuntime,**config['gains'],output=args.run_root/'feedback')
original_factory=runner._make_policy
instances=[]


def factory(name,protocol,seed,**kwargs):
    policy=BilateralTemporalPolicy(protocol) if name=='goal_greedy' and mode=='candidate' else original_factory(name,protocol,seed,**kwargs)
    instances.append(policy)
    if mode=='candidate':
        act=policy.act
        def recorded_act(observation,pointgoal):
            action=act(observation,pointgoal)
            index=policy._blocks-1;dest=args.run_root/'online_inputs';dest.mkdir(exist_ok=True)
            arrays=dict(left_rgb=observation.stereo_left_rgb,right_rgb=observation.stereo_right_rgb,
                position_world_xy=observation.position_world_xy,yaw_world_rad=observation.yaw_world_rad,
                quaternion_world_wxyz_history=observation.quaternion_world_wxyz_history,
                proprio_history=observation.proprio_history,action_history=observation.action_history,
                pointgoal_base_xy=np.asarray(pointgoal),request=action,observed_obstacle_xy=policy.last_observed)
            for eye in ('left','right'):
                for field in ('uv','depth_m','points_world','ncc','uniqueness'):
                    arrays[f'{eye}_{field}']=policy.last_prediction[eye][field]
            np.savez_compressed(dest/f'frame_{index:03d}.npz',**arrays)
            return action
        policy.act=recorded_act
    return policy


class RecordRuntime(base):
    @staticmethod
    def _create_markers(stage, episode, oracle_path, gf, usd_geom):
        actual = base._create_markers(stage, episode, oracle_path, gf, usd_geom)
        snapshot = hide_diagnostic_markers(stage, usd_geom)
        (args.run_root/'marker_visibility_at_creation.json').write_text(json.dumps(snapshot, indent=2)+'\n')
        return actual

    def __init__(self,config):
        super().__init__(replace(config,diagnostic_physics_trace=True))
        self._capture_index=0;self._reference_drawn=False

    def _capture(self):
        import omni.usd
        from pxr import UsdGeom
        visibility = marker_visibility(omni.usd.get_context().get_stage(), UsdGeom)
        assert all(r['computed_visibility'] == 'invisible' for r in visibility.values())
        self._marker_checks = getattr(self, '_marker_checks', []) + [dict(capture=self._capture_index, markers=visibility)]
        step=super()._capture();index=self._capture_index;self._capture_index+=1
        if mode=='candidate':
            import omni.usd
            from bev_ground_truth import read_camera_depth
            from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose
            stage=omni.usd.get_context().get_stage();position,q=self._robot.get_world_pose()
            dest=args.run_root/'offline_truth';dest.mkdir(exist_ok=True)
            # The policy is never handed this directory or a depth object.
            np.savez_compressed(dest/f'frame_{index:03d}.npz',
                left_depth_m=read_camera_depth(self._left_camera),right_depth_m=read_camera_depth(self._right_camera),
                left_camera_usd_world=get_cam_pose(stage,LEFT_CAMERA_PATH),right_camera_usd_world=get_cam_pose(stage,RIGHT_CAMERA_PATH),
                root_position_world_m=position,root_quaternion_world_wxyz=q,
                simulation_time_s=step.simulation_time_s,physics_step_index=step.physics_step_index)
        return step

    def execute(self,action):
        if mode=='candidate' and instances[-1]._path_xy is not None and not self._reference_drawn:
            import omni.usd
            from pxr import Gf,UsdGeom
            curve=UsdGeom.BasisCurves.Get(omni.usd.get_context().get_stage(),'/World/BenchmarkMarkers/OraclePath')
            path=instances[-1]._path_xy
            curve.GetCurveVertexCountsAttr().Set([len(path)])
            curve.GetPointsAttr().Set([Gf.Vec3f(float(x),float(y),.06) for x,y in path])
            self._reference_drawn=True
        return super().execute(action)

    def close(self,*positional,**keywords):
        if hasattr(self,'_marker_checks'):
            (args.run_root/'marker_visibility_each_capture.json').write_text(json.dumps(self._marker_checks, indent=2)+'\n')
        if hasattr(self,'_robot'):
            np.savez_compressed(args.run_root/'physics.npz',**self.physics_diagnostics)
        if instances and mode=='candidate':
            p=instances[-1]
            (args.run_root/'vision_records.json').write_text(json.dumps(p.records,indent=2)+'\n')
            if p._path_xy is not None:np.savez(args.run_root/'vision_reference.npz',position_world_xy=p._path_xy,arc_m=p._path_s)
        return super().close(*positional,**keywords)


runner._make_policy=factory;runner.IsaacBenchmarkRuntime=RecordRuntime
original_child=runner._child_command
def child(*positional,**keywords):
    command=original_child(*positional,**keywords);command[1]=str(WORKER);return command
runner._child_command=child
args.run_root.mkdir(parents=True,exist_ok=True)
(args.run_root/'experiment_identity.json').write_text(json.dumps(dict(
    mode=mode,actual_policy=BilateralTemporalPolicy.policy_name if mode=='candidate' else 'goal_greedy',
    privileged=False,online_inputs=sorted(BilateralTemporalPolicy.online_input_permissions),
    location_and_attitude_are_benchmark_measurements_not_visual_odometry=True,
    map_or_oracle_reference_consumed=False,renderer_depth_online=False,
    benchmark_render_markers_hidden_before_initial_capture=True,render_boundary_version='actual_imageable_markers_hidden_v2',
    perception_gate_passed=json.loads((HERE/'offline_assessment.json').read_text())['accepted'],
    scope='predeclared bounded online development validation; no deployment or general success-rate claim',
    camera_rig='frozen',lighting='fixed_camera_fill_v1',roll_pid_gains=config['gains'],formal_budget_s=35.,
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (HERE/'policy.py',HERE/'geometry.py',HERE/'control.py',HERE/'render_boundary.py',WORKER,HERE/'PROTOCOL.md',WORKER.parent/'RENDER_BOUNDARY_ADDENDUM.md',ROLL/'roll_pid.py')}
),indent=2)+'\n')
raise SystemExit(runner.main())
