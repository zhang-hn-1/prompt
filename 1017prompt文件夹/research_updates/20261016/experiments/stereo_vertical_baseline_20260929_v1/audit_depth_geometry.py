"""Reproduce offline stereo-vs-renderer checks; renderer truth is never policy input."""

import json
from pathlib import Path

import numpy as np

from vertical_depth import vertical_stereo_depth
from visual_local_policy import occupied_cells
from visual_local_policy_v3 import project_external_depth, choose_external_heading


HERE = Path(__file__).resolve().parent


def _sample(path: Path, *, baseline_m: float, external: bool) -> dict:
    frame = np.load(path)
    sample = {"frame": path.stem, "time_s": float(frame["simulation_time_s"]), "cameras": {}}
    predicted_cells = []
    truth_cells = []
    for camera in ("left", "right"):
        predicted, valid = vertical_stereo_depth(
            frame["left_rgb"], frame["right_rgb"],
            anchor=camera, baseline_m=baseline_m,
        )
        truth = frame[f"{camera}_depth_m"]
        finite_truth = np.isfinite(truth) & (truth > 0)
        paired = valid & finite_truth
        abs_error = np.abs(predicted[paired] - truth[paired])
        sample["cameras"][camera] = {
            "valid_fraction": float(valid.mean()),
            "rendered_self_under_45cm_fraction": float((finite_truth & (truth < 0.45)).mean()),
            "paired_pixels": int(paired.sum()),
            "median_absolute_error_m": float(np.median(abs_error)) if len(abs_error) else None,
        }
        if external:
            attitude = frame["root_quaternion_world_wxyz"]
            predicted_cells.append(occupied_cells(project_external_depth(predicted, valid, camera, attitude)))
            truth_mask = finite_truth & (truth > 0.2) & (truth < 10.0)
            truth_cells.append(occupied_cells(project_external_depth(truth, truth_mask, camera, attitude)))
    if external:
        predicted_cells = np.concatenate(predicted_cells)
        truth_cells = np.concatenate(truth_cells)
        predicted_set = set(map(tuple, predicted_cells))
        truth_set = set(map(tuple, truth_cells))
        sample["projected_cells"] = {
            "predicted_unique": int(len(predicted_set)),
            "renderer_unique": int(len(truth_set)),
            "predicted_absent_from_renderer": int(len(predicted_set - truth_set)),
            "predicted_center_absent_from_renderer": int(sum(
                x < 2.5 and abs(y) < 0.6 for x, y in predicted_set - truth_set
            )),
        }
        if path.stem == "frame_000" and "e2" in str(path):
            inferred_heading, _ = choose_external_heading(np.array([8.9, 0.0]), predicted_cells)
            renderer_heading, _ = choose_external_heading(np.array([8.9, 0.0]), truth_cells)
            sample["initial_heading_from_stereo_rad"] = inferred_heading
            sample["initial_heading_from_renderer_rad"] = renderer_heading
    return sample


def main() -> None:
    result = {}
    for variant, baseline, external in (("frozen", 0.6, False), ("front_external", 0.2, True)):
        result[variant] = {}
        for scene in ("e1", "e2"):
            directory = HERE / f"depth_audit_{'control_golden_' if variant == 'frozen' else ''}{scene}_0001" / "depth_truth"
            if external:
                directory = HERE / f"depth_audit_front_external_{scene}" / "depth_truth"
            samples = [_sample(path, baseline_m=baseline, external=external) for path in sorted(directory.glob("*.npz"))]
            if not samples:
                raise RuntimeError(f"no offline samples: {directory}")
            result[variant][scene] = samples
    output = HERE / "depth_geometry_audit.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
