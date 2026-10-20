"""Audit candidate-clearance label balance on complete, separated episodes."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from curve_labels import STEP_M, candidate_centerline, classify_curve, clearance_grid


HERE = Path(__file__).resolve().parent
PILOT = HERE.parents[1]
sys.path.insert(0, str(PILOT))
from pilot import yaw_from_quaternion  # noqa: E402


OCCUPANCY = (Path("/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark")
             / "research/ball_wm/data/benchmark_v1/occupancy")
EPISODES = {
    "E0_direct_fit": (PILOT / "integrated_fill_e0b/depth_truth", "hard0", "fit"),
    "E1_direct_fit": (PILOT / "integrated_fill_e1/depth_truth", "e1", "fit"),
    "E2_direct_fit": (PILOT / "integrated_fill_e2/depth_truth", "e2", "fit"),
    "hard_0_0001_fit": (PILOT / "illuminated_replay_v1/hard_0_e1_0001/depth_truth", "hard0", "fit"),
    "hard_0_0002_fit": (PILOT / "illuminated_replay_v1/hard_0_e1_0002/depth_truth", "hard0", "fit"),
    "hard_0_0003_test": (HERE.parent / "bilateral_depth_h3/depth_truth", "hard0", "test"),
    "hard_0_0004_test": (HERE.parent / "bilateral_depth_h4/depth_truth", "hard0", "test"),
}
MAP_FILES = {
    "hard0": OCCUPANCY / "cluttered_hard_0_bodyband.npz",
    "e1": OCCUPANCY / "e1_detour_bodyband.npz",
    "e2": OCCUPANCY / "e2_corridor_bodyband.npz",
}
SIDES = ("straight", "left", "right")
LABELS = ("clear", "blocked", "unknown")


def load_map(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    with np.load(path) as archive:
        grid = np.asarray(archive["grid"], dtype=np.uint8)
        origin = np.asarray(archive["global_origin_xy"], dtype=np.float64)
        resolution = float(np.asarray(archive["resolution_m"]).reshape(-1)[0])
    return clearance_grid(grid, resolution), origin, resolution


def audit_episode(folder: Path, map_data: tuple[np.ndarray, np.ndarray, float]) -> dict:
    paths = sorted(folder.glob("frame_*.npz"))
    if not paths:
        raise FileNotFoundError(folder)
    clearance, origin, resolution = map_data
    counts = {side: {label: 0 for label in LABELS} for side in SIDES}
    records = []
    mixed_frames = 0
    all_blocked_or_unknown = 0
    for path in paths:
        with np.load(path) as frame:
            position = frame["root_position_world_m"][:2]
            yaw = yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
            time = float(frame["simulation_time_s"])
        result = {}
        for side in SIDES:
            curve = candidate_centerline(position, yaw, side)
            label = classify_curve(curve, clearance, origin, resolution)
            result[side] = {
                **label,
                "first_event_forward_m": round(label["first_event_sample"] * STEP_M, 2)
                if label["first_event_sample"] is not None else None,
            }
            counts[side][label["label"]] += 1
        labels = [result[side]["label"] for side in SIDES]
        mixed_frames += "clear" in labels and "blocked" in labels
        all_blocked_or_unknown += "clear" not in labels
        records.append({"frame": path.name, "time_s": time,
                        "position_world_xy": position.tolist(), "yaw_world_rad": yaw,
                        "candidates": result})
    return {"complete_frames": len(paths), "candidate_label_counts": counts,
            "frames_with_clear_and_blocked_choice": mixed_frames,
            "frames_without_any_clear_choice": all_blocked_or_unknown,
            "records": records}


def main() -> None:
    maps = {key: load_map(path) for key, path in MAP_FILES.items()}
    episodes = {name: audit_episode(folder, maps[map_key])
                for name, (folder, map_key, _) in EPISODES.items()}
    fit_mixed = sum(episodes[name]["frames_with_clear_and_blocked_choice"]
                    for name, (_, _, split) in EPISODES.items() if split == "fit")
    test_mixed = {name: episodes[name]["frames_with_clear_and_blocked_choice"]
                  for name, (_, _, split) in EPISODES.items() if split == "test"}
    viable = fit_mixed >= 15 and all(count >= 5 for count in test_mixed.values())
    result = {
        "protocol": str(HERE / "protocol.md"),
        "offline_truth_only": "scene body-band occupancy maps",
        "candidate_geometry_inputs": "current root XY/quaternion plus fixed curve definitions",
        "map_paths": {key: str(path) for key, path in MAP_FILES.items()},
        "fit_frames_with_both_clear_and_blocked": fit_mixed,
        "test_frames_with_both_clear_and_blocked": test_mixed,
        "dataset_viable_for_next_classifier": viable,
        "episodes": episodes,
    }
    (HERE / "label_audit.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    summary = {name: {key: value for key, value in data.items() if key != "records"}
               for name, data in episodes.items()}
    print(json.dumps({"fit_mixed": fit_mixed, "test_mixed": test_mixed,
                      "viable": viable, "episodes": summary}, indent=2))


if __name__ == "__main__":
    main()
