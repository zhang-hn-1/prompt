"""Verify the declared-input external-rig E2 run and its historical control."""

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
CASE = "episode_golden_e2_0001"


def _load(name: str):
    directory = ROOT / name
    bundle = directory / CASE
    return (
        directory,
        json.loads((bundle / "metrics.json").read_text()),
        json.loads((bundle / "environment.json").read_text()),
        json.loads((bundle / "collision_audit.json").read_text()),
        np.load(bundle / "states.npz"),
        json.loads((directory / "stereo_experiment_identity.json").read_text()),
    )


def main() -> None:
    control = _load("navigation_control_golden_e2_0001")
    candidate = _load("navigation_visual_v3_external_declared_e2_0001")
    _, cm, ce, cc, cs, ci = control
    _, vm, ve, vc, vs, vi = candidate
    assert cm["policy"] == "goal_greedy" and vm["policy"] == "stereo_external_v3"
    assert cm["episode_budget_s"] == vm["episode_budget_s"] == 35.0
    assert cm["evaluation_protocol_id"] == vm["evaluation_protocol_id"] == "ball_pointgoal_v1_2_contact_35s"
    assert ce["scene_sha256"] == ve["scene_sha256"]
    assert ce["episode_seed"] == ve["episode_seed"]
    assert ce["collision_mode"] == ve["collision_mode"] == "physical_contact_v2"
    assert ci["roll_pid_gains"] == vi["roll_pid_gains"]
    assert vi["rig_variant"] == "front_external"
    assert vi["rig_positions_base_link_m"] == [[0.48, 0.1, 0.1], [0.48, -0.1, 0.1]]
    assert not ve["privileged"]
    assert {"stereo_rgb", "quaternion_world_wxyz_history", "pointgoal_base_xy"}.issubset(
        set(ve["policy_online_input_permissions"])
    )
    for name, digest in vi["source_sha256"].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest
    np.testing.assert_allclose(cs["position_world_xy"][0], vs["position_world_xy"][0], atol=1e-4)
    np.testing.assert_allclose(cs["yaw_world_rad"][0], vs["yaw_world_rad"][0], atol=1e-4)
    assert cc["contact_measurement_available"] and vc["contact_measurement_available"]
    assert bool(cm["collision"]) == any(item["contact"] for item in cc["samples"])
    assert bool(vm["collision"]) == any(item["contact"] for item in vc["samples"])
    assert cm["success"] and not vm["success"] and vm["collision"]
    assert vm["runtime_error_message"] is None
    print(json.dumps({
        "control": {"success": bool(cm["success"]), "elapsed_s": cm["elapsed_s"]},
        "external_v3": {
            "success": bool(vm["success"]), "collision": bool(vm["collision"]),
            "elapsed_s": vm["elapsed_s"], "path_length_m": vm["path_length_m"],
            "termination_reason": vm["termination_reason"],
        },
    }, indent=2))


if __name__ == "__main__":
    main()
