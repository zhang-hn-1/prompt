"""Run a non-privileged stereo policy through the unchanged formal runner."""

import hashlib
import json
import os
import sys
from pathlib import Path

import scripts.run_pointgoal_benchmark as runner

from visual_local_policy import StereoLocalPolicy
from visual_local_policy_v2 import StereoLocalPolicyV2
from visual_local_policy_v3 import StereoExternalPolicyV3
from rig_variant import front_external_positions


HERE = Path(__file__).resolve().parent
ORACLE_EXPERIMENT = HERE.parent / "oracle_feasible_route_20260929_v9" / "gated_speed_012"
sys.path.insert(0, str(ORACLE_EXPERIMENT))
from roll_pid import install_runtime  # noqa: E402


args = runner._build_parser().parse_args()
mode = os.environ.get("STEREO_LOCAL_MODE", "visual")
if mode not in {"visual", "visual_v2", "visual_v3_external", "control"}:
    raise ValueError("STEREO_LOCAL_MODE must be visual, visual_v2, visual_v3_external, or control")
config = json.loads((ORACLE_EXPERIMENT / "selected_candidate.json").read_text())
runner.IsaacBenchmarkRuntime = install_runtime(
    runner.IsaacBenchmarkRuntime,
    **config["gains"],
    output=args.run_root / "feedback",
)
original_factory = runner._make_policy
instances = []


def policy_factory(name, protocol, seed, **kwargs):
    if mode == "visual_v3_external" and name == "goal_greedy":
        policy = StereoExternalPolicyV3(protocol)
    elif mode == "visual_v2" and name == "goal_greedy":
        policy = StereoLocalPolicyV2(protocol)
    elif mode == "visual" and name == "goal_greedy":
        policy = StereoLocalPolicy(protocol)
    else:
        policy = original_factory(name, protocol, seed, **kwargs)
    instances.append(policy)
    return policy


runner._make_policy = policy_factory
base_runtime = runner.IsaacBenchmarkRuntime


class RecordRuntime(base_runtime):
    def _initialize(self, scene_path, occupancy_path, episode):
        super()._initialize(scene_path, occupancy_path, episode)
        if mode == "visual_v3_external":
            import omni.usd
            from pxr import Gf, UsdGeom
            from collect_data_dubins import LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH, _prime_stereo_cameras

            stage = omni.usd.get_context().get_stage()
            for path, xyz in zip((LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH), front_external_positions()):
                ops = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
                if not ops or ops[0].GetOpType() != UsdGeom.XformOp.TypeTranslate:
                    raise RuntimeError(f"unexpected camera transform stack: {path}")
                ops[0].Set(Gf.Vec3d(*(float(v) for v in xyz)))
            self._world.render()
            _prime_stereo_cameras(self._world, self._left_camera, self._right_camera)

    def close(self, *close_args, **close_kwargs):
        # Isaac can terminate the interpreter during close; flush beforehand.
        if instances:
            records = getattr(instances[-1], "records", None)
            (args.run_root / f"visual_records_{self._active_episode_id}.json").write_text(
                json.dumps({"factory_instances": len(instances), "records": records}, indent=2) + "\n"
            )
        return super().close(*close_args, **close_kwargs)

    def reset(self, episode):
        self._active_episode_id = episode.episode_id
        return super().reset(episode)


runner.IsaacBenchmarkRuntime = RecordRuntime
original_child = runner._child_command


def child(*child_args, **child_kwargs):
    command = original_child(*child_args, **child_kwargs)
    command[1] = str(HERE / "run_worker.py")
    return command


runner._child_command = child
args.run_root.mkdir(parents=True, exist_ok=True)
(args.run_root / "stereo_experiment_identity.json").write_text(
    json.dumps({
        "mode": mode,
        "requested_runner_policy": "goal_greedy",
        "actual_policy": {
            "control": "goal_greedy",
            "visual": "stereo_local_v1",
            "visual_v2": "stereo_local_v2_bilateral_texture",
            "visual_v3_external": "stereo_external_v3",
        }[mode],
        "privileged": False,
        "low_level_controller": "frozen v9 roll PID",
        "roll_pid_gains": config["gains"],
        "rig_variant": "front_external" if mode == "visual_v3_external" else "frozen",
        "rig_positions_base_link_m": (
            [point.tolist() for point in front_external_positions()]
            if mode == "visual_v3_external" else [[0.1, 0.3, 0.1], [0.1, -0.3, 0.1]]
        ),
        "source_sha256": {
            name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
            for name in ("vertical_depth.py", "visual_local_policy.py", "visual_local_policy_v2.py", "visual_local_policy_v3.py", "rig_variant.py", "run_worker.py")
        },
    }, indent=2) + "\n"
)
raise SystemExit(runner.main())
