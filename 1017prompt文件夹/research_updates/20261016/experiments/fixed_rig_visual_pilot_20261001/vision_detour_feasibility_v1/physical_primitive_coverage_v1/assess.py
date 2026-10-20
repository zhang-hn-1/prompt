"""Assess exact replay, shared initial state, and physics/map-aligned contacts."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from primitive_measurement import PRIMITIVES, same_start


HERE = Path(__file__).resolve().parent
PROJECT = Path("/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm")
OLD = HERE.parent / "bilateral_depth_h4/episode_hard_0_e1_0004"


def replay_comparison() -> dict:
    fields = ("position_world_xy", "yaw_world_rad", "linear_velocity_world_m_s",
              "angular_velocity_world_rad_s", "joint_position_rad", "joint_velocity_rad_s",
              "applied_action", "simulation_time_s", "physics_step_index",
              "physical_contact", "collision")
    differences = {}
    with np.load(HERE / "b0_replay/states.npz") as current, np.load(OLD / "states.npz") as old:
        for key in fields:
            equal_shape = current[key].shape == old[key].shape
            differences[key] = {
                "shape_match": equal_shape,
                "max_abs_difference": float(np.max(np.abs(current[key].astype(float) - old[key].astype(float))))
                if equal_shape else None}
    current = json.loads((HERE / "b0_replay/contact_samples.json").read_text())["samples"]
    old = json.loads((OLD / "collision_audit.json").read_text())["samples"]
    contact_match = len(current) == len(old) and all(
        x["physics_step_index"] == y["physics_step_index"] and x["source"] == y["source"]
        and x["contact"] == y["contact"] and x["available"] == y["available"]
        and abs(x["simulation_time_s"] - y["simulation_time_s"]) <= 1e-6
        for x, y in zip(current, old))
    return {"passed": contact_match and all(d["shape_match"] and d["max_abs_difference"] <= 1e-6
                                             for d in differences.values()),
            "state_fields": differences, "contact_sequence_match": contact_match,
            "contact_samples": len(current)}


def occupied_cell_distances(xy: np.ndarray, grid: np.ndarray,
                            origin: np.ndarray, resolution: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact continuous XY distance to occupied cell centers and cell square areas.

    Neither is a physics mesh/contact measurement. Axis 0 is world X.
    """
    centers = origin + np.argwhere(grid == 1) * resolution
    tree = cKDTree(centers)
    center_distances = tree.query(xy)[0]
    area_distances = []
    half_diagonal = resolution / np.sqrt(2)
    for point, center_distance in zip(xy, center_distances):
        # Every possibly nearest square has its center within this bound.
        neighbors = tree.query_ball_point(point, center_distance + half_diagonal + 1e-9)
        deltas = np.maximum(np.abs(centers[neighbors] - point) - resolution / 2, 0)
        area_distances.append(np.linalg.norm(deltas, axis=1).min())
    return center_distances, np.asarray(area_distances)


def contact_geometry(case: str, grid: np.ndarray, origin: np.ndarray, resolution: float) -> dict:
    folder = HERE / case
    with np.load(folder / "physics.npz") as trace:
        xy = trace["position_world_xy"]
        t = trace["simulation_time_s"]
        physics_steps = trace["physics_step_index"]
    centered, area = occupied_cell_distances(xy, grid, origin, resolution)
    contact = json.loads((folder / "contact_samples.json").read_text())["samples"]
    first = next((sample for sample in contact if sample["contact"]), None)
    indices = np.rint((xy - origin) / resolution).astype(int)
    inside = np.all((indices >= 0) & (indices < np.asarray(grid.shape)), axis=1)
    known = np.zeros(len(xy), dtype=bool)
    for k in np.flatnonzero(inside):
        known[k] = grid[tuple(indices[k])] != 2
    result = {"physics_samples": len(xy), "unknown_or_outside_samples": int((~known).sum()),
              "min_distance_to_occupied_cell_centers_m": float(centered.min()),
              "min_distance_to_occupied_cell_areas_m": float(area.min()),
              "contact_measurement_source": "actual physical-contact-v2; aligned by exact physics step"}
    if first is not None:
        matches = np.flatnonzero(physics_steps == first["physics_step_index"])
        if len(matches) != 1:
            raise ValueError("contact has no unique aligned physics state")
        k = int(matches[0])
        if abs(t[k] - first["simulation_time_s"]) > 1e-6:
            raise ValueError("physics/contact time mismatch")
        result["first_contact"] = {
            "time_s": float(t[k]), "physics_step": int(physics_steps[k]), "xy": xy[k].tolist(),
            "distance_to_occupied_cell_centers_m": float(centered[k]),
            "distance_to_occupied_cell_areas_m": float(area[k]),
            "previous_no_contact_time_s": float(t[k - 1]) if k else None,
            "previous_no_contact_cell_center_distance_m": float(centered[k - 1]) if k else None}
    else:
        result["first_contact"] = None
    return result


def main() -> None:
    replay = replay_comparison()
    if not replay["passed"]:
        raise ValueError("B0 replay mismatch")
    initial = json.loads((HERE / "b0_replay/initial_execution.json").read_text())
    with np.load(PROJECT / "data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz") as occ:
        grid = occ["grid"]
        origin = occ["global_origin_xy"]
        resolution = float(occ["resolution_m"])
    results = {}
    for case in ("b0_replay", *PRIMITIVES):
        result = json.loads((HERE / case / "result.json").read_text())
        current_initial = json.loads((HERE / case / "initial_execution.json").read_text())
        result["same_complete_initial_state"] = same_start(initial, current_initial)
        if not result["same_complete_initial_state"]:
            raise ValueError("initial state mismatch " + case)
        result["map_contact_geometry"] = contact_geometry(case, grid, origin, resolution)
        results[case] = result
    moving = [results[name] for name in PRIMITIVES if name != "stop"]
    coverage = (any(r["local_outcome"] == "survived_moving" for r in moving)
                and any(r["local_outcome"] == "contact" for r in moving))
    data = {"replay_validation": replay, "results": results,
            "moving_survival_and_contact_coverage_at_6s": coverage,
            "moving_survivors": [r["case"] for r in moving if r["local_outcome"] == "survived_moving"],
            "not_navigation_success": True, "fresh_held_out_scenes": 0,
            "map_resolution_m": resolution}
    (HERE / "assessment.json").write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    compact = {name: {"outcome": r["local_outcome"], "contact_s": r["first_contact_time_s"],
                       "displacement_m": r["net_displacement_m"],
                       "first_contact_cell_area_distance_m": r["map_contact_geometry"]["first_contact"]["distance_to_occupied_cell_areas_m"]
                       if r["map_contact_geometry"]["first_contact"] else None}
               for name, r in results.items()}
    print(json.dumps({"coverage": coverage, "results": compact}, indent=2))


if __name__ == "__main__":
    main()
