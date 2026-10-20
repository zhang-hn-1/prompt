"""Offline depth-oracle visibility of actual future swept-envelope samples.

Actual future poses are labels in this retrospective audit, never prediction
inputs. Sample visibility is not an RGB model score or a full safety proof.
"""

import json
import sys
from pathlib import Path

import numpy as np

from primitive_measurement import PRIMITIVES

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from pilot import FOCAL_PX, CX, CY  # noqa: E402


def envelope_points(root_centers: np.ndarray) -> np.ndarray:
    points = []
    angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    for center in root_centers:
        for dz in (-0.25, 0.0, 0.25):
            radius_xy = np.sqrt(0.4**2 - dz**2)
            offsets = np.vstack(([0.0, 0.0],
                                 radius_xy * np.column_stack((np.cos(angles), np.sin(angles)))))
            points.append(center + np.column_stack((offsets, np.full(len(offsets), dz))))
    return np.concatenate(points)


def eye_evidence(points: np.ndarray, depth: np.ndarray, pose: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rotation = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
    optical = (points - pose[:3, 3]) @ rotation
    z = optical[:, 2]
    uv = np.rint(np.column_stack((FOCAL_PX * optical[:, 0] / np.maximum(z, 1e-9) + CX,
                                  FOCAL_PX * optical[:, 1] / np.maximum(z, 1e-9) + CY))).astype(int)
    inside = (z > 0.05) & (uv[:, 0] >= 0) & (uv[:, 0] < depth.shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < depth.shape[0])
    observed = np.full(len(points), np.nan)
    observed[inside] = depth[uv[inside, 1], uv[inside, 0]]
    known = inside & np.isfinite(observed) & (observed >= 0.45) & (observed <= 10.0)
    free = known & (z <= observed - 0.05)
    occluded = known & (z >= observed + 0.05)
    return inside, free, occluded


def main():
    result = {}
    with np.load(HERE / "b0_replay/offline_depth/frame_000.npz") as initial:
        root = initial["root_position_world_m"]
        for case in PRIMITIVES:
            with np.load(HERE / case / "physics.npz") as trace:
                points = envelope_points(trace["root_position_world_m"][::5])
            points = points[np.linalg.norm(points - root, axis=1) >= 0.55]
            if not len(points):
                result[case] = {"sampled_future_body_points_outside_initial_self_region": 0}
                continue
            eyes = {eye: eye_evidence(points, initial[f"{eye}_depth_m"], initial[f"{eye}_camera_usd_world"])
                    for eye in ("left", "right")}
            either_free = eyes["left"][1] | eyes["right"][1]
            result[case] = {
                "sampled_future_body_points_outside_initial_self_region": len(points),
                "either_eye_depth_oracle_free_fraction": float(either_free.mean()),
                "either_eye_geometric_fov_fraction": float((eyes["left"][0] | eyes["right"][0]).mean()),
                "eyes": {eye: {"geometric_fov_fraction": float(evidence[0].mean()),
                               "depth_oracle_free_fraction": float(evidence[1].mean()),
                               "depth_oracle_occluded_fraction": float(evidence[2].mean())}
                         for eye, evidence in eyes.items()}}
    data = {"scope": "retrospective sampled swept-volume visibility, not model accuracy or full clearance",
            "current_input": "only initial measured camera poses and initial rendered depths",
            "future_truth_role": "actual primitive root centers define offline query samples only",
            "envelope": "27 sampled points per pose on three ball cross-sections; 0.40 m radius",
            "self_exclusion_m": 0.55, "depth_free_margin_m": 0.05,
            "results": result}
    (HERE / "initial_visibility_audit.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({k: {"points": v["sampled_future_body_points_outside_initial_self_region"],
                          "free_fraction": v.get("either_eye_depth_oracle_free_fraction")}
                      for k, v in result.items()}, indent=2))


if __name__ == "__main__":
    main()
