"""Record paired RGB, renderer depth and poses; never feed truth to policy."""

import json
import os
import sys
from pathlib import Path

import numpy as np

import scripts.run_pointgoal_benchmark as runner


HERE = Path(__file__).resolve().parent
ORACLE_EXPERIMENT = HERE.parent / "oracle_feasible_route_20260929_v9" / "gated_speed_012"
sys.path.insert(0, str(ORACLE_EXPERIMENT))
from roll_pid import install_runtime  # noqa: E402
from rig_variant import front_external_positions, front_narrow_positions  # noqa: E402

args = runner._build_parser().parse_args()
rig_variant = os.environ.get("STEREO_RIG_VARIANT", "frozen")
if rig_variant not in {"frozen", "front_narrow", "front_external"}:
    raise ValueError("STEREO_RIG_VARIANT must be frozen, front_narrow, or front_external")
capture_stride = int(os.environ.get("STEREO_CAPTURE_STRIDE", "0"))
if capture_stride < 0:
    raise ValueError("STEREO_CAPTURE_STRIDE must be non-negative")
config = json.loads((ORACLE_EXPERIMENT / "selected_candidate.json").read_text())
base = install_runtime(
    runner.IsaacBenchmarkRuntime,
    **config["gains"],
    output=args.run_root / "feedback",
)


class DepthAuditRuntime(base):
    SELECTED = {0, 10, 30, 50, 70, 90, 110, 130}

    def __init__(self, *init_args, **init_kwargs):
        super().__init__(*init_args, **init_kwargs)
        self._audit_capture_index = 0

    def _initialize(self, scene_path, occupancy_path, episode):
        super()._initialize(scene_path, occupancy_path, episode)
        if rig_variant in {"front_narrow", "front_external"}:
            import omni.usd
            from pxr import Gf, UsdGeom
            from collect_data_dubins import LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH, _prime_stereo_cameras

            stage = omni.usd.get_context().get_stage()
            positions = (
                front_narrow_positions()
                if rig_variant == "front_narrow" else front_external_positions()
            )
            for path, xyz in zip((LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH), positions):
                ops = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
                if not ops or ops[0].GetOpType() != UsdGeom.XformOp.TypeTranslate:
                    raise RuntimeError(f"unexpected camera transform stack: {path}")
                ops[0].Set(Gf.Vec3d(*(float(v) for v in xyz)))
            self._world.render()
            _prime_stereo_cameras(self._world, self._left_camera, self._right_camera)

    def _capture(self):
        step = super()._capture()
        index = self._audit_capture_index
        self._audit_capture_index += 1
        if index in self.SELECTED or (capture_stride > 0 and index % capture_stride == 0):
            from bev_ground_truth import read_camera_depth
            from collect_data_dubins import LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH, get_cam_pose
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            left_depth = read_camera_depth(self._left_camera)
            right_depth = read_camera_depth(self._right_camera)
            left_rgb = step.observation.stereo_left_rgb
            right_rgb = step.observation.stereo_right_rgb
            if left_depth.shape != left_rgb.shape[:2] or right_depth.shape != right_rgb.shape[:2]:
                raise RuntimeError("renderer depth/RGB shapes differ")
            position, quaternion = self._robot.get_world_pose()
            output = args.run_root / "depth_truth"
            output.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(
                output / f"frame_{index:03d}.npz",
                left_rgb=left_rgb,
                right_rgb=right_rgb,
                left_depth_m=left_depth,
                right_depth_m=right_depth,
                left_camera_usd_world=get_cam_pose(stage, LEFT_CAMERA_PATH),
                right_camera_usd_world=get_cam_pose(stage, RIGHT_CAMERA_PATH),
                root_position_world_m=np.asarray(position),
                root_quaternion_world_wxyz=np.asarray(quaternion),
                simulation_time_s=np.asarray(step.simulation_time_s),
            )
        return step


runner.IsaacBenchmarkRuntime = DepthAuditRuntime
original_child = runner._child_command


def child(*child_args, **child_kwargs):
    command = original_child(*child_args, **child_kwargs)
    command[1] = str(HERE / "run_depth_audit_worker.py")
    return command


runner._child_command = child
args.run_root.mkdir(parents=True, exist_ok=True)
(args.run_root / "depth_truth_usage.json").write_text(
    json.dumps({
        "policy": "goal_greedy",
        "truth_role": "offline perception diagnosis only; runtime capture side-channel",
        "sample_capture_indices": sorted(DepthAuditRuntime.SELECTED),
        "capture_stride": capture_stride,
        "roll_pid_gains": config["gains"],
        "rig_variant": rig_variant,
        "rig_positions_base_link_m": (
            [point.tolist() for point in front_narrow_positions()]
            if rig_variant == "front_narrow" else
            [point.tolist() for point in front_external_positions()]
            if rig_variant == "front_external" else
            [[0.1, 0.3, 0.1], [0.1, -0.3, 0.1]]
        ),
    }, indent=2) + "\n"
)
raise SystemExit(runner.main())
