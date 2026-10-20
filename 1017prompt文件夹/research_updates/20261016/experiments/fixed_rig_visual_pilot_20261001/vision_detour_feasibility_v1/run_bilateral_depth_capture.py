"""Capture rendered depth beside the frozen RGB policy for offline audit only."""

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import scripts.run_pointgoal_benchmark as runner

from bilateral_shadow import BilateralShadowPolicy
from policy import MODEL


HERE = Path(__file__).resolve().parent
ORACLE_EXPERIMENT = HERE.parents[1] / "oracle_feasible_route_20260929_v9" / "gated_speed_012"
sys.path.insert(0, str(ORACLE_EXPERIMENT))
from roll_pid import install_runtime  # noqa: E402


args = runner._build_parser().parse_args()
config = json.loads((ORACLE_EXPERIMENT / "selected_candidate.json").read_text())
runner.IsaacBenchmarkRuntime = install_runtime(
    runner.IsaacBenchmarkRuntime, **config["gains"], output=args.run_root / "feedback",
)
original_factory = runner._make_policy
instances = []


def policy_factory(name, protocol, seed, **kwargs):
    if name == "goal_greedy":
        policy = BilateralShadowPolicy(protocol)
    else:
        policy = original_factory(name, protocol, seed, **kwargs)
    instances.append(policy)
    return policy


runner._make_policy = policy_factory
base_runtime = runner.IsaacBenchmarkRuntime


class DepthAuditRuntime(base_runtime):
    def __init__(self, *init_args, **init_kwargs):
        super().__init__(*init_args, **init_kwargs)
        self._audit_capture_index = 0

    def _capture(self):
        step = super()._capture()
        index = self._audit_capture_index
        self._audit_capture_index += 1
        if index % 2 == 0:
            import omni.usd
            from bev_ground_truth import read_camera_depth
            from collect_data_dubins import LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH, get_cam_pose

            stage = omni.usd.get_context().get_stage()
            left_rgb = step.observation.stereo_left_rgb
            right_rgb = step.observation.stereo_right_rgb
            left_depth = read_camera_depth(self._left_camera)
            right_depth = read_camera_depth(self._right_camera)
            if left_depth.shape != left_rgb.shape[:2] or right_depth.shape != right_rgb.shape[:2]:
                raise RuntimeError("rendered depth and RGB disagree on image shape")
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

    def close(self, *close_args, **close_kwargs):
        if instances:
            (args.run_root / f"vision_records_{self._active_episode_id}.json").write_text(
                json.dumps({"factory_instances": len(instances), "records": instances[-1].records}, indent=2) + "\n"
            )
        return super().close(*close_args, **close_kwargs)

    def reset(self, episode):
        self._active_episode_id = episode.episode_id
        self._audit_capture_index = 0
        return super().reset(episode)


runner.IsaacBenchmarkRuntime = DepthAuditRuntime
original_child = runner._child_command


def child(*child_args, **child_kwargs):
    command = original_child(*child_args, **child_kwargs)
    command[1] = str(HERE / "run_bilateral_depth_capture.py")
    return command


runner._child_command = child
args.run_root.mkdir(parents=True, exist_ok=True)
(args.run_root / "depth_truth_usage.json").write_text(json.dumps({
    "online_inputs": "RGB, proprioception, pointgoal, perfect XY/yaw only",
    "depth_role": "offline labels only, captured after RGB observation",
    "capture_stride": 2,
    "camera_rig": "frozen",
    "policy": BilateralShadowPolicy.policy_name,
    "model_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
    "source_sha256": {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                      for name in ("policy.py", "policy_continuous.py", "bilateral_shadow.py", "run_bilateral_depth_capture.py")},
}, indent=2) + "\n")
raise SystemExit(runner.main())
