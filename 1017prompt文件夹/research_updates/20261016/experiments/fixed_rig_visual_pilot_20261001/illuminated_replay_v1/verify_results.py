"""Independent identity and outcome checks for the lit fixed-camera replay."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def main() -> None:
    plain = read_json(HERE / "results.json")
    added = read_json(HERE / "augmented" / "results.json")
    assert plain["gate_passed"] is False
    assert added["gate_passed"] is False
    assert set(added["source_paths"]) == {"e0", "e1", "e2", "h1", "h2"}
    for holdout in ("e1", "e2"):
        assert holdout not in added["folds"][holdout]["training_scenes"]
        assert set(added["folds"][holdout]["training_scenes"]) == (
            set(added["source_paths"]) - {holdout})
        assert added["folds"][holdout]["held_out_scene"] == holdout

    runs = {}
    for name in ("navigation_visual_v2_e2_dark_control", "navigation_visual_v2_e2"):
        root = HERE / name
        episode = root / "episode_golden_e2_0001"
        runs[name] = {
            "identity": read_json(root / "stereo_experiment_identity.json"),
            "environment": read_json(episode / "environment.json"),
            "metrics": read_json(episode / "metrics.json"),
            "initial_state": np.load(episode / "states.npz")["position_world_xy"][0],
            "video_exists": (episode / "stereo_view.mp4").exists(),
        }
    dark, lit = runs.values()
    assert dark["identity"]["source_sha256"] == lit["identity"]["source_sha256"]
    assert dark["identity"]["roll_pid_gains"] == lit["identity"]["roll_pid_gains"]
    assert dark["identity"]["rig_positions_base_link_m"] == lit["identity"]["rig_positions_base_link_m"]
    assert dark["identity"]["rig_variant"] == lit["identity"]["rig_variant"] == "frozen"
    for key in ("manifest_sha256", "scene_sha256", "occupancy_sha256", "evaluation_protocol_id",
                "episode_budget_s", "collision_mode"):
        assert dark["environment"][key] == lit["environment"][key], key
    assert dark["environment"]["lighting_profile"] == "benchmark_sun_v1"
    assert lit["environment"]["lighting_profile"] == "fixed_camera_fill_v1"
    assert np.allclose(dark["initial_state"], lit["initial_state"], atol=1e-6)
    for run in (dark, lit):
        assert run["metrics"]["termination_reason"] == "collision"
        assert run["metrics"]["success"] == 0
        assert run["metrics"]["collision"] == 1
        assert run["video_exists"]

    audit = {
        "same_controller_source": True,
        "same_camera_rig": True,
        "same_scene_manifest_occupancy_and_rules": True,
        "same_initial_xy": True,
        "lighting_profile_is_intended_intervention": True,
        "both_online_runs_physical_collision": True,
        "plain_gate_passed": plain["gate_passed"],
        "augmented_gate_passed": added["gate_passed"],
        "dark_elapsed_s": dark["metrics"]["elapsed_s"],
        "lit_elapsed_s": lit["metrics"]["elapsed_s"],
    }
    (HERE / "verification.json").write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
