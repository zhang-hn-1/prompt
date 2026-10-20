from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from bev_ground_truth import BEVConfig, GlobalOccupancy
from trajectory_map import (
    NavigationTaskOverlay,
    TrajectoryRecord,
    _world_to_pixel,
    reconstruct_observed_occupancy,
    render_navigation_task_map,
    render_trajectory_map,
)
from plot_trajectory_map import load_scene_overlay


def _occupancy() -> GlobalOccupancy:
    return GlobalOccupancy(
        grid=np.array(
            [
                [0, 1, 2],
                [1, 0, 0],
            ],
            dtype=np.uint8,
        ),
        global_origin_xy=np.array([0.0, 0.0]),
        resolution_m=1.0,
    )


def test_render_trajectory_map_writes_png_with_world_y_up_orientation(tmp_path: Path):
    output = tmp_path / "trajectory_map.png"
    trajectory = TrajectoryRecord(
        label="ep0000",
        points_xy=np.array([[0.5, 0.5], [1.5, 1.5]], dtype=np.float64),
        collision_points_xy=np.array([[1.5, 1.5]], dtype=np.float64),
    )

    result = render_trajectory_map(output, _occupancy(), [trajectory])

    assert result == output
    assert output.is_file()
    with Image.open(output) as image:
        assert image.format == "PNG"
        assert image.size == (1200, 900)
        assert image.mode == "RGB"
        pixels = np.asarray(image)
    # The rendered image contains both map colors and the trajectory overlay.
    assert len(np.unique(pixels.reshape(-1, 3), axis=0)) > 8


def test_three_path_audit_uses_blue_purple_orange_and_start_goal_markers(tmp_path: Path):
    output = tmp_path / "three_path_audit.png"
    trajectories = (
        TrajectoryRecord(
            label="geometric A*",
            points_xy=np.asarray(((0.2, 0.3), (1.7, 0.3))),
            color_rgb=(35, 105, 210),
            draw_endpoint_markers=False,
        ),
        TrajectoryRecord(
            label="kinodynamic reference",
            points_xy=np.asarray(((0.2, 0.8), (1.7, 0.8))),
            color_rgb=(142, 68, 173),
            draw_endpoint_markers=False,
        ),
        TrajectoryRecord(
            label="actual",
            points_xy=np.asarray(((0.2, 1.3), (1.7, 1.3))),
            color_rgb=(230, 126, 34),
            end_color_rgb=(204, 45, 146),
        ),
    )

    render_trajectory_map(output, _occupancy(), trajectories, title="three paths")

    with Image.open(output) as image:
        colors = {rgb for _, rgb in image.convert("RGB").getcolors(maxcolors=1_000_000)}
    assert (35, 105, 210) in colors
    assert (142, 68, 173) in colors
    assert (230, 126, 34) in colors
    assert (35, 170, 80) in colors
    assert (204, 45, 146) in colors


def test_navigation_task_map_returns_complete_layer_manifest(tmp_path: Path):
    occupancy = GlobalOccupancy(
        grid=np.zeros((20, 20), dtype=np.uint8),
        global_origin_xy=np.asarray((0.0, 0.0)),
        resolution_m=0.1,
    )
    inflated = occupancy.grid.copy()
    inflated[8:12, 9:11] = 1
    overlay = NavigationTaskOverlay(
        inflated_grid=inflated,
        start_cell=(2, 2),
        start_yaw_rad=0.0,
        goal_cell=(17, 17),
        oracle_path_cells=np.asarray(((2, 2), (2, 17), (17, 17))),
        metrics=("difficulty: E1", "detour ratio: 1.20"),
    )

    output, layers = render_navigation_task_map(
        tmp_path / "navigation_task.png",
        occupancy,
        overlay,
        title="Task E1",
    )

    assert output.is_file()
    assert set(layers) == {
        "obstacles",
        "inflated",
        "start_heading",
        "goal",
        "straight_line",
        "oracle_path",
        "metrics",
    }


def test_world_to_pixel_keeps_world_x_right_and_world_y_up():
    world_min = np.array([-1.0, -1.0])
    world_max = np.array([1.0, 1.0])
    low = _world_to_pixel(np.array([0.0, -0.5]), world_min, world_max, 10.0, 20.0, 100.0)
    high = _world_to_pixel(np.array([0.5, 0.5]), world_min, world_max, 10.0, 20.0, 100.0)

    assert high[0] > low[0]
    assert high[1] < low[1]


def test_render_trajectory_map_rejects_invalid_trajectory_shape(tmp_path: Path):
    with pytest.raises(ValueError, match=r"points_xy must have shape \(N, 2\)"):
        TrajectoryRecord(
            label="bad",
            points_xy=np.zeros((3, 3), dtype=np.float64),
            collision_points_xy=np.empty((0, 2), dtype=np.float64),
        )


def test_render_trajectory_map_does_not_leave_partial_file_on_validation_error(tmp_path: Path):
    output = tmp_path / "trajectory_map.png"
    with pytest.raises(ValueError, match="at least one trajectory"):
        render_trajectory_map(output, _occupancy(), [])
    assert not output.exists()
    assert not output.with_suffix(output.suffix + ".partial").exists()


def test_reconstruct_observed_occupancy_places_local_bev_in_world_frame():
    config = BEVConfig(extent_m=0.4, resolution_m=0.1)
    frame = np.zeros(config.shape, dtype=np.uint8)
    frame[config.center[0], config.center[1]] = 1

    reconstructed = reconstruct_observed_occupancy(
        [frame],
        positions_xy=np.array([[2.0, -1.0]], dtype=np.float64),
        yaws=np.array([0.0], dtype=np.float64),
        config=config,
    )

    center_rows, center_cols = np.where(reconstructed.grid == 1)
    assert len(center_rows) == 1
    np.testing.assert_allclose(
        reconstructed.global_origin_xy
        + np.array([center_rows[0], center_cols[0]]) * reconstructed.resolution_m,
        [2.0, -1.0],
        atol=1e-8,
    )


def test_reconstruct_observed_occupancy_preserves_occupied_on_same_frame_quantization():
    config = BEVConfig(extent_m=0.4, resolution_m=0.1)
    frame = np.zeros(config.shape, dtype=np.uint8)
    frame[0, 0] = 1

    reconstructed = reconstruct_observed_occupancy(
        [frame],
        positions_xy=np.array([[0.0, 0.0]], dtype=np.float64),
        yaws=np.array([0.17907078125461823], dtype=np.float64),
        config=config,
    )

    assert np.count_nonzero(reconstructed.grid == 1) == 1


def test_load_scene_overlay_reads_saved_states_and_geometry(tmp_path: Path):
    episode = tmp_path / "scene_easy_0" / "ep0000"
    episode.mkdir(parents=True)
    (episode / "metadata.json").write_text(
        '{"bev_extent_m": 0.4, "bev_resolution_m": 0.1}',
        encoding="utf-8",
    )
    positions = np.array([[0.0, 0.0, 0.4], [0.1, 0.0, 0.4], [0.2, 0.0, 0.4]], dtype=np.float32)
    np.savez(
        episode / "states.npz",
        pos=positions,
        quat=np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (3, 1)),
        stereo_physics_step=np.array([1, 3], dtype=np.int64),
        collision=np.array([False, True]),
    )
    frame0 = np.zeros((4, 4), dtype=np.uint8)
    frame1 = np.zeros((4, 4), dtype=np.uint8)
    frame1[2, 2] = 1
    Image.fromarray(frame0).save(episode / "geometry_bev_000000.png")
    Image.fromarray(frame1).save(episode / "geometry_bev_000001.png")

    occupancy, records = load_scene_overlay(episode.parent)

    assert occupancy.grid.dtype == np.uint8
    assert len(records) == 1
    np.testing.assert_allclose(records[0].points_xy[:, 0], [0.0, 0.1, 0.2])
    np.testing.assert_allclose(records[0].collision_points_xy, [[0.2, 0.0]])
    assert np.count_nonzero(occupancy.grid == 1) == 1
