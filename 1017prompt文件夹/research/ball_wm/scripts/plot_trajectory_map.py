"""Generate scene-level trajectory maps from saved local BEV episodes.

The collector now writes this visualization directly.  This utility is useful
for older datasets whose scene-level global occupancy map was not persisted:
it fuses each saved ``geometry_bev`` frame back into world coordinates and
renders the recorded state paths over that observed map.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

import numpy as np
from PIL import Image

from bev_ground_truth import BEVConfig, GlobalOccupancy
from collect_data_dubins import _yaw_from_wxyz
from trajectory_map import (
    TrajectoryRecord,
    reconstruct_observed_occupancy,
    render_trajectory_map,
)


_GEOMETRY_PATTERN = re.compile(r"^geometry_bev_(\d{6})\.png$")


def _scene_episode_dirs(scene_dir: Path) -> tuple[Path, ...]:
    episodes = tuple(sorted(path for path in scene_dir.glob("ep*") if path.is_dir()))
    if not episodes:
        raise ValueError(f"no episode directories found under {scene_dir}")
    return episodes


def _read_geometry_frames(episode_dir: Path) -> list[np.ndarray]:
    paths: list[tuple[int, Path]] = []
    for path in episode_dir.glob("geometry_bev_*.png"):
        match = _GEOMETRY_PATTERN.match(path.name)
        if match:
            paths.append((int(match.group(1)), path))
    paths.sort()
    if not paths:
        raise ValueError(f"no geometry_bev frames found under {episode_dir}")
    frames: list[np.ndarray] = []
    expected_id = 0
    for frame_id, path in paths:
        if frame_id != expected_id:
            raise ValueError(
                f"geometry_bev frame IDs under {episode_dir} must start at 0 and be consecutive"
            )
        with Image.open(path) as image:
            frames.append(np.asarray(image.convert("L"), dtype=np.uint8).copy())
        expected_id += 1
    return frames


def _load_episode(
    episode_dir: Path,
    config: BEVConfig,
) -> tuple[TrajectoryRecord, list[np.ndarray], np.ndarray, np.ndarray]:
    states_path = episode_dir / "states.npz"
    metadata_path = episode_dir / "metadata.json"
    if not states_path.is_file() or not metadata_path.is_file():
        raise ValueError(f"episode is missing states.npz or metadata.json: {episode_dir}")
    with np.load(states_path, allow_pickle=False) as states:
        positions = np.asarray(states["pos"], dtype=np.float64)
        quaternions = np.asarray(states["quat"], dtype=np.float64)
        stereo_steps = np.asarray(states["stereo_physics_step"], dtype=np.int64)
        collisions = np.asarray(states["collision"], dtype=bool)
    if positions.ndim != 2 or positions.shape[1] < 2 or quaternions.shape != (len(positions), 4):
        raise ValueError(f"invalid pose arrays in {states_path}")
    if stereo_steps.ndim != 1 or collisions.shape != stereo_steps.shape:
        raise ValueError(f"invalid stereo arrays in {states_path}")
    if np.any(stereo_steps < 1) or np.any(stereo_steps > len(positions)):
        raise ValueError(f"stereo physics steps are outside state range in {states_path}")
    stereo_indices = stereo_steps - 1
    stereo_positions = positions[stereo_indices, :2]
    stereo_yaws = np.asarray(
        [_yaw_from_wxyz(quaternion) for quaternion in quaternions[stereo_indices]],
        dtype=np.float64,
    )
    frames = _read_geometry_frames(episode_dir)
    if len(frames) != len(stereo_positions):
        raise ValueError(
            f"geometry frame count {len(frames)} does not match stereo state count "
            f"{len(stereo_positions)} in {episode_dir}"
        )
    record = TrajectoryRecord(
        label=episode_dir.name,
        points_xy=positions[:, :2],
        collision_points_xy=stereo_positions[collisions],
    )
    return record, frames, stereo_positions, stereo_yaws


def load_scene_overlay(scene_dir: str | Path) -> tuple[GlobalOccupancy, list[TrajectoryRecord]]:
    """Load one scene's saved episodes and reconstruct its observed map."""

    scene_path = Path(scene_dir)
    episodes = _scene_episode_dirs(scene_path)
    with (episodes[0] / "metadata.json").open(encoding="utf-8") as handle:
        metadata = json.load(handle)
    config = BEVConfig(
        extent_m=float(metadata["bev_extent_m"]),
        resolution_m=float(metadata["bev_resolution_m"]),
    )
    records: list[TrajectoryRecord] = []
    maps: list[np.ndarray] = []
    frame_positions: list[np.ndarray] = []
    frame_yaws: list[np.ndarray] = []
    for episode in episodes:
        record, frames, positions, yaws = _load_episode(episode, config)
        records.append(record)
        maps.extend(frames)
        frame_positions.extend(positions)
        frame_yaws.extend(yaws)
    occupancy = reconstruct_observed_occupancy(
        maps,
        np.asarray(frame_positions, dtype=np.float64),
        np.asarray(frame_yaws, dtype=np.float64),
        config,
    )
    return occupancy, records


def _scene_dirs(root: Path) -> tuple[Path, ...]:
    if any(path.is_dir() and path.name.startswith("ep") for path in root.iterdir()):
        return (root,)
    scenes = tuple(sorted(path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")))
    if not scenes:
        raise ValueError(f"no scene directories found under {root}")
    return scenes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "data_root",
        type=Path,
        help="one scene directory or a root containing scene directories",
    )
    parser.add_argument("--output_name", default="trajectory_map.png")
    args = parser.parse_args(argv)
    if not args.data_root.is_dir():
        parser.error(f"data root does not exist: {args.data_root}")
    for scene in _scene_dirs(args.data_root):
        occupancy, records = load_scene_overlay(scene)
        output = render_trajectory_map(
            scene / args.output_name,
            occupancy,
            records,
            title=f"Robot trajectory — {scene.name} (observed local BEV)",
        )
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
