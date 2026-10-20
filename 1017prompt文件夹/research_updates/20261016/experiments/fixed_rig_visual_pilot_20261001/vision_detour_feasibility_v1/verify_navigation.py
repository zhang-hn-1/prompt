"""Verify frozen runtime identity, observed trigger behavior and physical outcomes."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
BASELINE = HERE.parent


def load(path: Path) -> dict:
    return json.loads(path.read_text())


def run_data(folder: str, episode_id: str) -> dict:
    root = HERE / folder
    episode = root / f"episode_{episode_id}"
    records = load(root / f"vision_records_{episode_id}.json")["records"]
    metrics = load(episode / "metrics.json")
    environment = load(episode / "environment.json")
    identity = load(root / "experiment_identity.json")
    with np.load(episode / "states.npz") as states:
        xy = states["position_world_xy"].copy()
    with np.load(episode / "actions.npz") as actions:
        action_blocks = actions["action_blocks"].copy()
    assert (episode / "stereo_view.mp4").exists()
    assert (episode / "third_person.mp4").exists()
    assert environment["lighting_profile"] == "fixed_camera_fill_v1"
    assert environment["episode_budget_s"] == 35.0
    assert environment["collision_mode"] == "physical_contact_v2"
    assert identity["camera_rig"] == "frozen"
    assert identity["privileged"] is False
    assert len(records) == metrics["action_blocks"]
    return {"records": records, "metrics": metrics, "environment": environment,
            "identity": identity, "xy": xy, "actions": action_blocks}


def main() -> None:
    runs = {
        "e0": run_data("e0_continuous", "golden_e0_0001"),
        "e1": run_data("e1_continuous", "golden_e1_0001"),
        "e2": run_data("e2_continuous", "golden_e2_0001"),
        "h3": run_data("hard_0_e1_0003", "hard_0_e1_0003"),
        "h4": run_data("hard_0_e1_0004", "hard_0_e1_0004"),
    }
    identities = [run["identity"] for run in runs.values()]
    assert all(identity == identities[0] for identity in identities)
    assert all(runs[scene]["metrics"]["success"] == 1 for scene in ("e0", "e1", "e2"))
    assert all(runs[scene]["metrics"]["collision"] == 0 for scene in ("e0", "e1", "e2"))
    assert all(runs[scene]["metrics"]["termination_reason"] == "success" for scene in ("e0", "e1", "e2"))
    assert all(runs[scene]["metrics"]["success"] == 0 for scene in ("h3", "h4"))
    assert all(runs[scene]["metrics"]["termination_reason"] == "collision" for scene in ("h3", "h4"))
    assert not any(item["detour_triggered"] for item in runs["e0"]["records"])
    assert not any(item["detour_triggered"] for item in runs["e2"]["records"])
    assert runs["e1"]["records"][1]["detour_triggered"]
    assert not any(item["detour_triggered"] for item in runs["h3"]["records"])
    assert runs["h4"]["records"][1]["detour_triggered"]

    action_equal = {}
    state_equal = {}
    for scene, original in (("e0", "integrated_fill_e0b"), ("e2", "integrated_fill_e2")):
        episode_id = f"golden_{scene}_0001"
        with np.load(BASELINE / original / f"episode_{episode_id}" / "actions.npz") as actions:
            action_equal[scene] = bool(np.array_equal(runs[scene]["actions"], actions["action_blocks"]))
        with np.load(BASELINE / original / f"episode_{episode_id}" / "states.npz") as states:
            state_equal[scene] = bool(np.array_equal(runs[scene]["xy"], states["position_world_xy"]))
        assert action_equal[scene] and state_equal[scene]

    e1_xy = runs["e1"]["xy"]
    crossed = np.flatnonzero(e1_xy[:, 0] >= 0)
    assert len(crossed) and np.min(e1_xy[:, 1]) < -1.9
    audit = {
        "policy_sha256": identities[0]["source_sha256"],
        "model_sha256": identities[0]["model_sha256"],
        "golden": {scene: {
            "termination_reason": runs[scene]["metrics"]["termination_reason"],
            "elapsed_s": runs[scene]["metrics"]["elapsed_s"],
            "path_length_m": runs[scene]["metrics"]["path_length_m"],
            "goal_progress_m": runs[scene]["metrics"]["goal_progress_m"],
            "collision": runs[scene]["metrics"]["collision"],
            "trigger_action_block": next((i for i, item in enumerate(runs[scene]["records"])
                                          if item["detour_triggered"]), None),
        } for scene in ("e0", "e1", "e2")},
        "extra_whole_sequences": {scene: {
            "termination_reason": runs[scene]["metrics"]["termination_reason"],
            "elapsed_s": runs[scene]["metrics"]["elapsed_s"],
            "path_length_m": runs[scene]["metrics"]["path_length_m"],
            "goal_progress_m": runs[scene]["metrics"]["goal_progress_m"],
            "collision": runs[scene]["metrics"]["collision"],
            "trigger_action_block": next((i for i, item in enumerate(runs[scene]["records"])
                                          if item["detour_triggered"]), None),
        } for scene in ("h3", "h4")},
        "golden_no_trigger_baseline_action_equal": action_equal,
        "golden_no_trigger_baseline_xy_equal": state_equal,
        "e1_min_world_y_m": float(np.min(e1_xy[:, 1])),
        "e1_first_cross_x0_world_y_m": float(e1_xy[crossed[0], 1]),
        "same_source_model_camera_and_pid_all_runs": True,
        "original_35s_physical_contact_rules_all_runs": True,
    }
    (HERE / "navigation_verification.json").write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps({"golden": audit["golden"], "extra": audit["extra_whole_sequences"],
                      "e1_cross_y_m": audit["e1_first_cross_x0_world_y_m"]}, indent=2))


if __name__ == "__main__":
    main()
