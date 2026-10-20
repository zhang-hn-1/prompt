"""Re-run the frozen pixel pilot on integrated pre-init illuminated frames."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import torch


HERE = Path(__file__).resolve().parent
PILOT_DIR = HERE.parent
sys.path.insert(0, str(PILOT_DIR))
import pilot  # noqa: E402


SCENES = {
    "e0": PILOT_DIR / "integrated_fill_e0b" / "depth_truth",
    "e1": PILOT_DIR / "integrated_fill_e1" / "depth_truth",
    "e2": PILOT_DIR / "integrated_fill_e2" / "depth_truth",
}


def illuminated_scene_paths(scene: str) -> list[Path]:
    paths = sorted(SCENES[scene].glob("frame_*.npz"))
    if not paths:
        raise FileNotFoundError(SCENES[scene])
    return paths


def coverage(dataset: dict) -> dict:
    labels = dataset["labels"]
    central = dataset["central"]
    times = dataset["times"]
    early = times <= 2.0
    wall = central & (labels == pilot.OBSTACLE)
    floor = central & (labels == pilot.FREE)
    return {
        "frames_by_eye": len(labels),
        "known_fraction": float(np.mean(labels != pilot.UNKNOWN)),
        "central_wall_samples": int(wall.sum()),
        "central_wall_0_to_2s_samples": int(wall[early].sum()),
        "central_floor_samples": int(floor.sum()),
    }


def main() -> None:
    torch.set_num_threads(min(4, torch.get_num_threads()))
    pilot.scene_paths = illuminated_scene_paths
    datasets = {scene: pilot.load_scene(scene) for scene in SCENES}
    result = {
        "protocol": str(HERE / "protocol.md"),
        "lighting_profile": "fixed_camera_fill_v1",
        "device": "cpu",
        "seed": pilot.SEED,
        "source_paths": {scene: [str(path) for path in illuminated_scene_paths(scene)] for scene in SCENES},
        "coverage": {scene: coverage(dataset) for scene, dataset in datasets.items()},
        "folds": {},
    }
    HERE.mkdir(parents=True, exist_ok=True)
    for holdout in ("e1", "e2"):
        result["folds"][holdout] = pilot.train_fold(holdout, torch.device("cpu"), HERE, datasets)
        (HERE / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    e1, e2 = result["folds"]["e1"], result["folds"]["e2"]
    result["gate_passed"] = bool(
        all(fold["unknown_recall"] >= 0.90 and fold["free_false_obstacle_rate"] <= 0.05
            for fold in (e1, e2))
        and e1["central_wall_0_to_2s_obstacle_recall"] is not None
        and e1["central_wall_0_to_2s_obstacle_recall"] >= 0.80
    )
    (HERE / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"gate_passed": result["gate_passed"],
                      "e1": {key: e1[key] for key in ("unknown_recall", "free_false_obstacle_rate", "central_wall_0_to_2s_obstacle_recall")},
                      "e2": {key: e2[key] for key in ("unknown_recall", "free_false_obstacle_rate", "central_ground_false_obstacle_rate")}}, indent=2))


if __name__ == "__main__":
    main()
