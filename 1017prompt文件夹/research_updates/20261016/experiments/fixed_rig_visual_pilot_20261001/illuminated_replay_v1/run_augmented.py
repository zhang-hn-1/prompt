"""One frozen additional-data candidate for the lit fixed-rig pixel pilot."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import torch


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import pilot  # noqa: E402


SCENES = {
    "e0": HERE.parent / "integrated_fill_e0b" / "depth_truth",
    "e1": HERE.parent / "integrated_fill_e1" / "depth_truth",
    "e2": HERE.parent / "integrated_fill_e2" / "depth_truth",
    "h1": HERE / "hard_0_e1_0001" / "depth_truth",
    "h2": HERE / "hard_0_e1_0002" / "depth_truth",
}


def scene_paths(scene: str) -> list[Path]:
    paths = sorted(SCENES[scene].glob("frame_*.npz"))
    if not paths:
        raise FileNotFoundError(SCENES[scene])
    return paths


def split_scenes(holdout: str) -> tuple[list[str], list[str]]:
    if holdout not in {"e1", "e2"}:
        raise ValueError(holdout)
    return sorted(SCENES.keys() - {holdout}), [holdout]


def main() -> None:
    torch.set_num_threads(min(4, torch.get_num_threads()))
    pilot.scene_paths = scene_paths
    pilot.split_scenes = split_scenes
    datasets = {scene: pilot.load_scene(scene) for scene in SCENES}
    output = HERE / "augmented"
    output.mkdir(parents=True, exist_ok=True)
    result = {
        "protocol": str(HERE / "augmentation_protocol.md"),
        "lighting_profile": "fixed_camera_fill_v1",
        "device": "cpu",
        "seed": pilot.SEED,
        "source_paths": {scene: [str(p) for p in scene_paths(scene)] for scene in SCENES},
        "folds": {},
    }
    for holdout in ("e1", "e2"):
        result["folds"][holdout] = pilot.train_fold(holdout, torch.device("cpu"), output, datasets)
        (output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    e1, e2 = result["folds"]["e1"], result["folds"]["e2"]
    result["gate_passed"] = bool(
        all(x["unknown_recall"] >= 0.90 and x["free_false_obstacle_rate"] <= 0.05
            for x in (e1, e2))
        and e1["central_wall_0_to_2s_obstacle_recall"] >= 0.80
    )
    (output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"gate_passed": result["gate_passed"],
                      "e1_wall": e1["central_wall_0_to_2s_obstacle_recall"],
                      "e1_unknown": e1["unknown_recall"],
                      "e1_false_obstacle": e1["free_false_obstacle_rate"],
                      "e2_unknown": e2["unknown_recall"],
                      "e2_false_obstacle": e2["free_false_obstacle_rate"]}, indent=2))


if __name__ == "__main__":
    main()
