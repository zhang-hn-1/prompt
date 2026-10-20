"""Check the frozen golden comparisons and emit a compact result table."""

import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
EPISODES = ("e0", "e1", "e2")
PROTOCOL = "ball_pointgoal_v1_2_contact_35s"
V1_SOURCE_HASHES = {
    "vertical_depth.py": "72878b97a41aee070d8f289999fe8fdb9b026fead4a7ae2ae73fa332c0541159",
    "visual_local_policy.py": "206fa75c7120d8b980e2dc24740be6dab2ae04548cc6ef06a243d536d5428f8c",
}
V2_SOURCE_HASHES = {
    "vertical_depth.py": "44613f9f7e24e9adb13fc14bfa68bd02c87b2b221d16f5f185b18fdcecb3029c",
    "visual_local_policy.py": "d1d529d7217662cd717448f6d6f92a32c6305ecf7a155bdb37b01a03c92e3ca2",
    "visual_local_policy_v2.py": "c0f293d84c6aa505f8f467fe08c7f1db5d4226e7fd55f87e3716349e2596fc30",
}


def load(episode: str, mode: str):
    suffix = "_logged" if episode == "e1" and mode == "visual" else ""
    directory = ROOT / f"navigation_{mode}_golden_{episode}_0001{suffix}"
    bundle = directory / f"episode_golden_{episode}_0001"
    metrics = json.loads((bundle / "metrics.json").read_text())
    environment = json.loads((bundle / "environment.json").read_text())
    contact = json.loads((bundle / "collision_audit.json").read_text())
    states = np.load(bundle / "states.npz")
    actions = np.load(bundle / "actions.npz")
    identity = json.loads((directory / "stereo_experiment_identity.json").read_text())
    return directory, metrics, environment, contact, states, actions, identity


def main() -> None:
    summary = {}
    for episode in EPISODES:
        summary[episode] = {}
        runs = {mode: load(episode, mode) for mode in ("control", "visual")}
        control = runs["control"]
        visual = runs["visual"]
        assert control[2]["evaluation_protocol_id"] == visual[2]["evaluation_protocol_id"] == PROTOCOL
        assert control[2]["scene_sha256"] == visual[2]["scene_sha256"]
        assert control[2]["episode_seed"] == visual[2]["episode_seed"]
        np.testing.assert_allclose(control[4]["position_world_xy"][0], visual[4]["position_world_xy"][0], atol=1e-4)
        np.testing.assert_allclose(control[4]["yaw_world_rad"][0], visual[4]["yaw_world_rad"][0], atol=1e-4)
        assert not control[2]["privileged"] and not visual[2]["privileged"]
        assert "stereo_rgb" in visual[2]["policy_online_input_permissions"]
        for mode, (directory, metrics, environment, contact, states, actions, identity) in runs.items():
            assert metrics["runtime_error_message"] is None
            assert metrics["episode_budget_s"] == 35.0
            assert environment["collision_mode"] == "physical_contact_v2"
            assert contact["contact_measurement_available"]
            assert bool(metrics["collision"]) == any(sample["contact"] for sample in contact["samples"])
            assert identity["roll_pid_gains"] == control[6]["roll_pid_gains"]
            if mode == "visual":
                assert metrics["policy"] == "stereo_local_v1"
                for name, digest in V1_SOURCE_HASHES.items():
                    assert identity["source_sha256"][name] == digest
            else:
                assert metrics["policy"] == "goal_greedy"
            summary[episode][mode] = {
                "success": bool(metrics["success"]),
                "collision": bool(metrics["collision"]),
                "timeout": bool(metrics["timeout"]),
                "elapsed_s": metrics["elapsed_s"],
                "path_length_m": metrics["path_length_m"],
                "termination_reason": metrics["termination_reason"],
                "bundle": str(directory),
            }
        assert not np.allclose(control[5]["action_blocks"][0], visual[5]["action_blocks"][0])
    assert summary["e0"]["visual"]["success"]
    assert summary["e1"]["visual"]["collision"]
    assert summary["e2"]["control"]["success"] and summary["e2"]["visual"]["collision"]
    candidate = ROOT / "navigation_visual_v2_golden_e2_0001"
    candidate_bundle = candidate / "episode_golden_e2_0001"
    candidate_metrics = json.loads((candidate_bundle / "metrics.json").read_text())
    candidate_environment = json.loads((candidate_bundle / "environment.json").read_text())
    candidate_contact = json.loads((candidate_bundle / "collision_audit.json").read_text())
    candidate_identity = json.loads((candidate / "stereo_experiment_identity.json").read_text())
    candidate_states = np.load(candidate_bundle / "states.npz")
    assert candidate_metrics["policy"] == "stereo_local_v2_bilateral_texture"
    assert candidate_environment["evaluation_protocol_id"] == PROTOCOL
    assert candidate_environment["scene_sha256"] == visual[2]["scene_sha256"]
    assert candidate_environment["episode_seed"] == visual[2]["episode_seed"]
    np.testing.assert_allclose(candidate_states["position_world_xy"][0], visual[4]["position_world_xy"][0], atol=1e-4)
    assert candidate_metrics["collision"] and not candidate_metrics["success"]
    assert any(sample["contact"] for sample in candidate_contact["samples"])
    # V2 was a frozen historical run. Compare its saved identity to its archived
    # digests rather than to files modified by later isolated experiments.
    for name, digest in V2_SOURCE_HASHES.items():
        assert candidate_identity["source_sha256"][name] == digest
    summary["e2"]["visual_v2"] = {
        "success": False,
        "collision": True,
        "elapsed_s": candidate_metrics["elapsed_s"],
        "path_length_m": candidate_metrics["path_length_m"],
        "termination_reason": candidate_metrics["termination_reason"],
        "bundle": str(candidate),
    }
    (ROOT / "comparison_verified.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
