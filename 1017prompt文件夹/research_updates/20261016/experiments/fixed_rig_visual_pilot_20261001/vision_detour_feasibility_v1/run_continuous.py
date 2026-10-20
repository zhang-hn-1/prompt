"""Run the continuous RGB-triggered detour through the formal runner."""

import hashlib
import json
import sys
from pathlib import Path

import scripts.run_pointgoal_benchmark as runner

from policy import MODEL
from policy_continuous import ContinuousVisionDetourPolicy


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
        policy = ContinuousVisionDetourPolicy(protocol)
    else:
        policy = original_factory(name, protocol, seed, **kwargs)
    instances.append(policy)
    return policy


runner._make_policy = policy_factory
base_runtime = runner.IsaacBenchmarkRuntime


class RecordRuntime(base_runtime):
    def close(self, *close_args, **close_kwargs):
        if instances:
            records = getattr(instances[-1], "records", None)
            (args.run_root / f"vision_records_{self._active_episode_id}.json").write_text(
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
    command[1] = str(HERE / "run_continuous.py")
    return command


runner._child_command = child
args.run_root.mkdir(parents=True, exist_ok=True)
(args.run_root / "experiment_identity.json").write_text(json.dumps({
    "actual_policy": ContinuousVisionDetourPolicy.policy_name,
    "privileged": False,
    "camera_rig": "frozen",
    "roll_pid_gains": config["gains"],
    "source_sha256": {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                      for name in ("policy.py", "policy_continuous.py", "run_continuous.py")},
    "model_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
}, indent=2) + "\n")
raise SystemExit(runner.main())
