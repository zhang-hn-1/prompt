"""Summarize schema-1.1/1.2 ball robot data, including nested scene roots."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image

try:
    from .dataset_paths import discover_episode_paths
except ImportError:
    from dataset_paths import discover_episode_paths


def summarize(root: str | Path) -> dict[str, object]:
    root = Path(root)
    episodes = discover_episode_paths(root)
    per_scene: dict[str, int] = {}
    episode_details: list[dict[str, object]] = []
    total_states = 0
    total_frames = 0
    total_collisions = 0
    total_elevation_frames = 0
    total_elevation_valid_cells = 0
    total_elevation_cells = 0
    elevation_min_m = np.inf
    elevation_max_m = -np.inf
    action_min = np.array([np.inf, np.inf])
    action_max = np.array([-np.inf, -np.inf])
    bev_counts = np.zeros(3, dtype=np.int64)

    for episode in episodes:
        with np.load(episode / "states.npz", allow_pickle=False) as states:
            metadata = json.loads((episode / "metadata.json").read_text())
            positions = np.asarray(states["pos"], dtype=np.float64)
            collision = np.asarray(states["collision"], dtype=bool)
            applied = np.asarray(states["applied_action"], dtype=np.float32)
            state_count = int(len(states["physics_step"]))
            frame_count = int(len(states["stereo_frame_id"]))
            collision_count = int(np.count_nonzero(collision))
            trajectory_length = float(np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1).sum())
            displacement = float(np.linalg.norm(positions[-1, :2] - positions[0, :2]))
        scene = metadata.get("scene", str(episode))
        elevation_enabled = bool(metadata.get("elevation_enabled", False))
        elevation_frames = 0
        elevation_valid_cells = 0
        elevation_cells = 0
        episode_elevation_min = np.inf
        episode_elevation_max = -np.inf
        for valid_path in sorted(episode.glob("height_valid_bev_*.png")):
            frame_id = valid_path.stem.rsplit("_", 1)[-1]
            min_path = episode / f"height_min_bev_{frame_id}.npy"
            max_path = episode / f"height_max_bev_{frame_id}.npy"
            if not min_path.exists() or not max_path.exists():
                continue
            valid = np.asarray(Image.open(valid_path).convert("L"), dtype=np.uint8) == 1
            height_min = np.asarray(np.load(min_path), dtype=np.float32)
            height_max = np.asarray(np.load(max_path), dtype=np.float32)
            values = np.concatenate((height_min[valid], height_max[valid]))
            elevation_frames += 1
            elevation_valid_cells += int(np.count_nonzero(valid))
            elevation_cells += int(valid.size)
            if values.size:
                episode_elevation_min = min(episode_elevation_min, float(values.min()))
                episode_elevation_max = max(episode_elevation_max, float(values.max()))
        if elevation_frames:
            elevation_enabled = True
            total_elevation_frames += elevation_frames
            total_elevation_valid_cells += elevation_valid_cells
            total_elevation_cells += elevation_cells
            elevation_min_m = min(elevation_min_m, episode_elevation_min)
            elevation_max_m = max(elevation_max_m, episode_elevation_max)
        per_scene[scene] = per_scene.get(scene, 0) + 1
        total_states += state_count
        total_frames += frame_count
        total_collisions += collision_count
        action_min = np.minimum(action_min, applied.min(axis=0))
        action_max = np.maximum(action_max, applied.max(axis=0))
        episode_details.append(
            {
                "path": str(episode),
                "scene": scene,
                "episode_index": metadata.get("episode_index"),
                "schema_version": metadata.get("schema_version"),
                "state_samples": state_count,
                "stereo_frames": frame_count,
                "collision_frames": collision_count,
                "collision_rate": collision_count / max(1, frame_count),
                "trajectory_length_m": trajectory_length,
                "displacement_m": displacement,
                "trajectory_map": str(episode.parent / "trajectory_map.png"),
                "elevation_enabled": elevation_enabled,
                "elevation_frames": elevation_frames,
                "elevation_valid_cell_fraction": elevation_valid_cells / max(1, elevation_cells),
                "elevation_min_m": None if not elevation_frames else episode_elevation_min,
                "elevation_max_m": None if not elevation_frames else episode_elevation_max,
            }
        )

    for episode in episodes[:20]:
        for path in episode.glob("geometry_bev_*.png"):
            bev = np.asarray(Image.open(path).convert("L"))
            for value in range(3):
                bev_counts[value] += int(np.count_nonzero(bev == value))

    return {
        "episodes": len(episodes),
        "per_scene": per_scene,
        "episodes_detail": episode_details,
        "state_samples": total_states,
        "stereo_frames": total_frames,
        "collision_frames": total_collisions,
        "collision_rate": total_collisions / max(1, total_frames),
        "elevation_enabled_episodes": sum(
            bool(detail["elevation_enabled"]) for detail in episode_details
        ),
        "elevation_frames": total_elevation_frames,
        "elevation_valid_cell_fraction": total_elevation_valid_cells / max(1, total_elevation_cells),
        "elevation_min_m": None if total_elevation_frames == 0 else elevation_min_m,
        "elevation_max_m": None if total_elevation_frames == 0 else elevation_max_m,
        "applied_action_min": None if not episodes else action_min.tolist(),
        "applied_action_max": None if not episodes else action_max.tolist(),
        "bev_class_counts_sample": bev_counts.tolist(),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="optional JSON manifest path; the same summary is still printed",
    )
    args = parser.parse_args(argv)
    summary = summarize(args.root)
    payload = json.dumps(summary, indent=2, ensure_ascii=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
