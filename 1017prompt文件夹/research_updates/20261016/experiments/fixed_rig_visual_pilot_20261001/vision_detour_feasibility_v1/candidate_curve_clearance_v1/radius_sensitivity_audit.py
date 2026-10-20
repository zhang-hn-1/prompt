"""Post-hoc 0.40 m physical-radius comparison; frozen 0.60 m gate remains binding."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from curve_labels import candidate_centerline, classify_curve
from run_label_audit import EPISODES, HERE, MAP_FILES, SIDES, load_map, yaw_from_quaternion


def audit_episode(folder: Path, map_data: tuple[np.ndarray, np.ndarray, float],
                  threshold_m: float) -> dict:
    clearance, origin, resolution = map_data
    counts = {side: {label: 0 for label in ("clear", "blocked", "unknown")}
              for side in SIDES}
    mixed = 0
    no_clear = 0
    for path in sorted(folder.glob("frame_*.npz")):
        with np.load(path) as frame:
            xy = frame["root_position_world_m"][:2]
            yaw = yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
        labels = []
        for side in SIDES:
            points = candidate_centerline(xy, yaw, side)
            label = classify_curve(points, clearance, origin, resolution,
                                   required_clearance_m=threshold_m)["label"]
            counts[side][label] += 1
            labels.append(label)
        mixed += "clear" in labels and "blocked" in labels
        no_clear += "clear" not in labels
    return {"candidate_label_counts": counts,
            "frames_with_clear_and_blocked_choice": mixed,
            "frames_without_any_clear_choice": no_clear}


def main() -> None:
    maps = {key: load_map(path) for key, path in MAP_FILES.items()}
    results = {}
    for threshold in (0.4, 0.6):
        results[f"{threshold:.2f}_m"] = {
            name: audit_episode(folder, maps[map_key], threshold)
            for name, (folder, map_key, _) in EPISODES.items()
        }
    result = {
        "status": "post-hoc sensitivity only; not a revised label protocol or training gate",
        "thresholds": {"0.40_m": "ball shell radius with no buffer",
                       "0.60_m": "predeclared radius plus 0.20 m buffer"},
        "results": results,
    }
    (HERE / "radius_sensitivity_audit.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n")
    compact = {threshold: {
        name: {"mixed": record["frames_with_clear_and_blocked_choice"],
               "no_clear": record["frames_without_any_clear_choice"]}
        for name, record in episodes.items()}
        for threshold, episodes in results.items()}
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
