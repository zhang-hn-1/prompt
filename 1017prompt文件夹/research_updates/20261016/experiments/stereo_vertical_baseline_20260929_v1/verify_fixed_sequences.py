"""Check identity and completeness of the unchanged-camera diagnostic runs."""

import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent


def main():
    for scene, expected_count in (("e2", 71), ("e1", 30)):
        name = f"golden_{scene}_0001"
        current = ROOT / f"fixed_sequence_{name}"
        historical = ROOT / f"navigation_control_{name}"
        episode = f"episode_{name}"
        metrics = json.loads((current / episode / "metrics.json").read_text())
        environment = json.loads((current / episode / "environment.json").read_text())
        previous_environment = json.loads((historical / episode / "environment.json").read_text())
        metadata = json.loads((current / "depth_truth_usage.json").read_text())
        contact = json.loads((current / episode / "collision_audit.json").read_text())
        frames = sorted((current / "depth_truth").glob("frame_*.npz"))
        assert metrics["policy"] == "goal_greedy"
        assert metrics["episode_budget_s"] == 35.0
        assert environment["evaluation_protocol_id"] == "ball_pointgoal_v1_2_contact_35s"
        assert environment["collision_mode"] == "physical_contact_v2"
        assert environment["scene_sha256"] == previous_environment["scene_sha256"]
        assert environment["episode_seed"] == previous_environment["episode_seed"]
        assert metadata["rig_variant"] == "frozen" and metadata["capture_stride"] == 2
        assert metadata["rig_positions_base_link_m"] == [[0.1, 0.3, 0.1], [0.1, -0.3, 0.1]]
        assert len(frames) == expected_count
        assert all(int(path.stem.split("_")[-1]) == 2 * index for index, path in enumerate(frames))
        times = [float(np.load(path)["simulation_time_s"]) for path in frames]
        np.testing.assert_allclose(np.diff(times), 0.2, atol=1e-5)
        assert bool(metrics["collision"]) == any(item["contact"] for item in contact["samples"])
        print(scene, "frames", len(frames), "time", metrics["elapsed_s"], "success", bool(metrics["success"]))


if __name__ == "__main__":
    main()
