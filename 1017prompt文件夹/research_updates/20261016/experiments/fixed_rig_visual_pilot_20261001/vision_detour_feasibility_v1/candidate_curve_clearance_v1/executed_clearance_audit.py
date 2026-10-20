"""Compare frozen-map clearance with the actual E1 success and hard-0 failure.

This is a post-hoc diagnostic, not a collision label or a change to the
predeclared 0.60 m candidate-curve gate.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from run_label_audit import HERE, MAP_FILES, clearance_grid


PILOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
from spatial_evidence_audit import frozen_e1_curve  # noqa: E402

STATE_FILES = {
    "E1_success_actual": PILOT / "vision_detour_feasibility_v1/bilateral_depth_detour_e1/episode_golden_e1_0001/states.npz",
    "hard_0_0004_collision_actual": PILOT / "vision_detour_feasibility_v1/bilateral_depth_h4/episode_hard_0_e1_0004/states.npz",
}


def map_clearance(points: np.ndarray, map_path: Path) -> dict:
    with np.load(map_path) as archive:
        grid = archive["grid"]
        origin = archive["global_origin_xy"]
        resolution = float(archive["resolution_m"])
    distance = clearance_grid(grid, resolution)
    indices = np.rint((points - origin) / resolution).astype(np.int64)
    inside = np.all((indices >= 0) & (indices < np.asarray(grid.shape)), axis=1)
    known = np.zeros(len(points), dtype=bool)
    values = np.full(len(points), np.nan)
    for k in np.flatnonzero(inside):
        i, j = indices[k]
        if grid[i, j] != 2:
            known[k] = True
            values[k] = distance[i, j]
    finite = np.isfinite(values)
    if not finite.any():
        raise ValueError("No finite map-clearance samples")
    nearest = int(np.nanargmin(values))
    return {
        "samples": len(points),
        "known_samples": int(known.sum()),
        "unknown_or_outside_samples": int((~known).sum()),
        "min_frozen_map_center_clearance_m": float(values[nearest]),
        "min_clearance_sample_index": nearest,
        "min_clearance_world_xy": points[nearest].tolist(),
        "known_samples_below_0p60_m": int(np.sum(values < 0.6)),
        "known_samples_below_0p40_m": int(np.sum(values < 0.4)),
    }


def main() -> None:
    result = {}
    for name, path in STATE_FILES.items():
        with np.load(path) as state:
            xy = np.asarray(state["position_world_xy"], dtype=np.float64)
            duration = float(state["simulation_time_s"][-1])
        map_key = "e1" if name.startswith("E1") else "hard0"
        result[name] = {"state_path": str(path), "duration_s": duration,
                        **map_clearance(xy, MAP_FILES[map_key])}
    planned, _ = frozen_e1_curve()
    result["E1_frozen_planned_curve"] = {
        "source": "frozen_e1_curve() reconstructed from saved detour trigger",
        **map_clearance(planned, MAP_FILES["e1"]),
    }
    result["interpretation_limit"] = (
        "Body-band occupancy distance to known occupied cell centers is not "
        "a physics-contact measurement; 0.60 m is the frozen buffered label "
        "threshold, not a necessary condition for success."
    )
    output = HERE / "executed_clearance_audit.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
