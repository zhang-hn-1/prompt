"""Single privileged Oracle candidate through the unchanged 35 s formal runner."""

from dataclasses import replace
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

import scripts.run_pointgoal_benchmark as runner
from reverse_route import ReverseEscapeOracle


HERE = Path(__file__).resolve().parent
ROLL = HERE.parents[2] / "oracle_feasible_route_20260929_v9/gated_speed_012"
sys.path.insert(0, str(ROLL))
from roll_pid import install_runtime  # noqa: E402

args = runner._build_parser().parse_args()
if args.episode_id != "hard_0_e1_0004":
    raise ValueError("sealed one-episode continuation probe")
assessment = json.loads((HERE / "assessment.json").read_text())
if not assessment["moving_survival_and_contact_coverage_at_6s"]:
    raise ValueError("local primitive coverage did not pass")
config = json.loads((ROLL / "selected_candidate.json").read_text())
base = install_runtime(runner.IsaacBenchmarkRuntime, **config["gains"], output=args.run_root / "feedback")
instances = []
original_factory = runner._make_policy


def factory(name, protocol, seed, **kwargs):
    if name == "goal_greedy":
        policy = ReverseEscapeOracle(protocol, args.data_root / "benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz")
        instances.append(policy)
        return policy
    return original_factory(name, protocol, seed, **kwargs)


class RecordRuntime(base):
    def __init__(self, config):
        super().__init__(replace(config, diagnostic_physics_trace=True))
        self._new_reference_drawn = False

    def execute(self, action):
        if instances and instances[-1]._path is not None and not self._new_reference_drawn:
            import omni.usd
            from pxr import Gf, UsdGeom
            curve = UsdGeom.BasisCurves.Get(omni.usd.get_context().get_stage(), "/World/BenchmarkMarkers/OraclePath")
            path = instances[-1]._path
            curve.GetCurveVertexCountsAttr().Set([len(path)])
            curve.GetPointsAttr().Set([Gf.Vec3f(float(x), float(y), 0.06) for x, y in path])
            self._new_reference_drawn = True
        return super().execute(action)

    def close(self, *positional, **keywords):
        if instances:
            policy = instances[-1]
            (args.run_root / "policy_records.json").write_text(json.dumps(policy.records, indent=2) + "\n")
            if policy._path is not None:
                np.savez(args.run_root / "reference_built_at_6s.npz", position_world_xy=policy._path,
                         arc_m=policy._arc)
        if hasattr(self, "_robot"):
            np.savez_compressed(args.run_root / "physics.npz", **self.physics_diagnostics)
        return super().close(*positional, **keywords)


runner._make_policy = factory
runner.IsaacBenchmarkRuntime = RecordRuntime
original_child = runner._child_command


def child(*positional, **keywords):
    command = original_child(*positional, **keywords)
    command[1] = str(HERE / "run_navigation.py")
    return command


runner._child_command = child
args.run_root.mkdir(parents=True, exist_ok=True)
(args.run_root / "experiment_identity.json").write_text(json.dumps({
    "actual_policy": ReverseEscapeOracle.policy_name,
    "privileged": True, "inputs": sorted(ReverseEscapeOracle.online_input_permissions),
    "scope": "one historical development episode; no visual or held-out generalization claim",
    "roll_pid_gains": config["gains"], "formal_budget_s": 35.0,
    "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (HERE / "reverse_route.py", HERE / "run_navigation.py",
                                HERE / "NAVIGATION_PROBE_PROTOCOL.md")}}, indent=2) + "\n")
raise SystemExit(runner.main())
