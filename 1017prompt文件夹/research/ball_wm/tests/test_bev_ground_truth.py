import sys
import inspect
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import bev_ground_truth as ground_truth

from bev_ground_truth import (
    AnalyticElevationCoverageError,
    AnalyticElevationUnsupportedError,
    BEVConfig,
    GlobalElevation,
    GlobalOccupancy,
    IsaacHeightProvider,
    IsaacOccupancyProvider,
    VisibilityHistory,
    _create_world_bbox_cache,
    _extract_depth_from_frame,
    _import_omap_binding,
    _omap_transform_arguments,
    _robot_collisions_disabled,
    _validate_external_scene_occupancy,
    build_global_elevation,
    build_analytic_global_map,
    crop_global_occupancy,
    crop_global_elevation,
    depth_to_visible_mask,
    is_spawn_free,
    omap_buffer_to_global_occupancy,
    obstacle_clearance,
    rasterize_analytic_primitives,
    rasterize_analytic_height_envelope,
    validate_elevation_coverage,
    warp_history_mask,
    world_points_to_local_bev,
)


def test_rasterize_height_cylinder_envelope_has_world_z_bounds():
    elevation = rasterize_analytic_height_envelope(
        [{"kind": "cylinder", "center_xy": (0.0, 0.0), "radius": 0.5,
          "z_min": 0.2, "z_max": 1.2}],
        global_origin_xy=(-1.0, -1.0), grid_shape=(20, 20), resolution_m=0.1,
    )
    assert isinstance(elevation, GlobalElevation)
    assert elevation.valid.dtype == np.uint8
    assert elevation.valid[10, 10] == 1
    assert elevation.valid[10, 10]
    assert elevation.height_min[10, 10] == pytest.approx(0.2)
    assert elevation.height_max[10, 10] == pytest.approx(1.2)
    assert not elevation.valid[0, 0]
    assert elevation.height_min[0, 0] == 0.0
    assert elevation.height_max[0, 0] == 0.0


def test_rasterize_height_sphere_uses_xy_distance_bounds():
    elevation = rasterize_analytic_height_envelope(
        [{"kind": "sphere", "center_xy": (0.0, 0.0), "center_z": 1.0, "radius": 0.5}],
        global_origin_xy=(-1.0, -1.0), grid_shape=(20, 20), resolution_m=0.1,
    )
    assert elevation.valid[10, 10]
    assert elevation.height_min[10, 10] == pytest.approx(0.5)
    assert elevation.height_max[10, 10] == pytest.approx(1.5)
    assert elevation.valid[10, 14]
    assert elevation.height_min[10, 14] == pytest.approx(0.7)
    assert elevation.height_max[10, 14] == pytest.approx(1.3)
    assert not elevation.valid[10, 16]


@pytest.mark.parametrize("kwargs", [
    {"height_min": np.zeros((2, 2), dtype=np.float32), "height_max": np.zeros((3, 2), dtype=np.float32), "valid": np.ones((2, 2), dtype=np.uint8)},
    {"height_min": np.zeros((2, 2), dtype=np.float32), "height_max": np.zeros((2, 2)), "valid": np.ones((2, 2), dtype=bool)},
    {"height_min": np.zeros((2, 2), dtype=np.float64), "height_max": np.zeros((2, 2), dtype=np.float64), "valid": np.ones((2, 2), dtype=np.uint8)},
    {"height_min": np.zeros((2, 2), dtype=np.float32), "height_max": np.zeros((2, 2), dtype=np.float32), "valid": np.ones((2, 2), dtype=bool)},
])
def test_global_elevation_rejects_shape_or_dtype_mismatch(kwargs):
    with pytest.raises(ValueError):
        GlobalElevation(global_origin_xy=(0.0, 0.0), resolution_m=0.1, **kwargs)


def test_crop_global_elevation_subtracts_base_height_and_rotates_like_occupancy():
    source = rasterize_analytic_height_envelope(
        [{"kind": "cylinder", "center_xy": (0.0, 0.0), "radius": 0.2,
          "z_min": 0.4, "z_max": 1.4}],
        global_origin_xy=(-4.0, -4.0), grid_shape=(80, 80), resolution_m=0.1,
    )
    height_min, height_max, valid = crop_global_elevation(
        source, base_position=(0.0, 0.0, 0.4), base_yaw=0.0, config=BEVConfig())
    assert valid[40, 40]
    assert height_min[40, 40] == pytest.approx(0.0)
    assert height_max[40, 40] == pytest.approx(1.0)
    assert height_min[0, 0] == 0.0 and height_max[0, 0] == 0.0 and not valid[0, 0]


def test_validate_elevation_coverage_rejects_free_cell_without_valid_height():
    config = BEVConfig()
    elevation = GlobalElevation(np.zeros((2, 2), dtype=np.float32), np.ones((2, 2), dtype=np.float32), np.array([[1, 0], [1, 1]], dtype=np.uint8), (0.0, 0.0), 0.1)
    occupancy = np.full((2, 2), config.occupied, dtype=np.uint8)
    occupancy[0, 1] = config.free
    with pytest.raises(ValueError, match="free occupancy"):
        validate_elevation_coverage(elevation, occupancy, config)


@pytest.mark.parametrize("grid_shape", [(2.5, 3), (True, 3), (2, 3.5)])
def test_height_rasterizer_rejects_non_integral_grid_shape(grid_shape):
    with pytest.raises(ValueError):
        rasterize_analytic_height_envelope([], (0.0, 0.0), grid_shape, 0.1)


@pytest.mark.parametrize("primitive", [
    {"kind": "cylinder", "center_xy": (0.0, 0.0), "radius": 0.0, "z_min": 0.0, "z_max": 1.0},
    {"kind": "sphere", "center_xy": (0.0, 0.0), "center_z": 0.0, "radius": np.nan},
    {"kind": "cone", "center_xy": (0.0, 0.0), "radius": 1.0},
])
def test_height_rasterizer_rejects_malformed_primitive(primitive):
    with pytest.raises(ValueError):
        rasterize_analytic_height_envelope([primitive], (0.0, 0.0), (2, 2), 0.1)


def test_omap_transform_uses_origin_relative_xy_bounds_and_zero_height_slice():
    """Isaac OMap UI treats bounds as origin-relative and generate2d scans at z=0."""

    origin, lower, upper = _omap_transform_arguments(
        robot_origin=np.array([2.0, -3.0, 0.45]),
        scene_minimum=np.array([-8.0, -9.0, -1.0]),
        scene_maximum=np.array([10.0, 11.0, 4.0]),
    )

    assert origin == (2.0, -3.0, 0.45)
    assert lower == (-10.0, -6.0, 0.0)
    assert upper == (8.0, 14.0, 0.0)


def test_robot_origin_prefers_base_link_pose_over_identity_root():
    """The ball's scan height lives on base_link; the robot root is often at z=0."""

    class Transform:
        def __init__(self, translation):
            self.translation = translation

        def ExtractTranslation(self):
            return self.translation

    class Cache:
        def __init__(self, stage):
            self.stage = stage

        def GetLocalToWorldTransform(self, prim):
            return Transform(prim.translation)

    class Prim:
        def __init__(self, translation=(0.0, 0.0, 0.0), valid=True):
            self.translation = translation
            self.valid = valid

        def IsValid(self):
            return self.valid

    class Stage:
        def __init__(self):
            self.prims = {
                "/World/BallRobot": Prim((0.0, 0.0, 0.0)),
                "/World/BallRobot/base_link": Prim((-0.01, -0.002, 0.4)),
            }

        def GetPrimAtPath(self, path):
            return self.prims.get(path, Prim(valid=False))

    class UsdGeom:
        @staticmethod
        def XformCache():
            return Cache(Stage())

    provider = IsaacOccupancyProvider(
        stage=Stage(),
        scene_root="/World/Scene",
        robot_root="/World/BallRobot",
        config=BEVConfig(),
    )

    origin = provider._robot_origin(UsdGeom)
    assert origin[2] == pytest.approx(0.4)


def test_robot_origin_falls_back_to_root_without_base_link():
    """Providers for robots without base_link must still resolve a usable origin."""

    class Transform:
        def __init__(self, translation):
            self.translation = translation

        def ExtractTranslation(self):
            return self.translation

    class Cache:
        def GetLocalToWorldTransform(self, prim):
            return Transform(prim.translation)

    class Prim:
        def __init__(self, translation=(0.0, 0.0, 0.45), valid=True):
            self.translation = translation
            self.valid = valid

        def IsValid(self):
            return self.valid

    class Stage:
        def __init__(self):
            self.prims = {"/World/Robot": Prim((0.0, 0.0, 0.45))}

        def GetPrimAtPath(self, path):
            return self.prims.get(path, Prim(valid=False))

    class UsdGeom:
        @staticmethod
        def XformCache():
            return Cache()

    provider = IsaacOccupancyProvider(
        stage=Stage(),
        scene_root="/World/Scene",
        robot_root="/World/Robot",
        config=BEVConfig(),
    )

    assert provider._robot_origin(UsdGeom)[2] == pytest.approx(0.45)


def test_occupancy_provider_accepts_physics_sync_callback():
    """OMap must re-sync PhysX after robot collisions are disabled."""

    calls = []

    def sync():
        calls.append(1)

    provider = IsaacOccupancyProvider(
        stage=object(),
        scene_root="/World/Scene",
        robot_root="/World/Robot",
        config=BEVConfig(),
        physics_sync=sync,
    )

    provider.physics_sync()
    assert calls == [1]


def test_external_scene_occupancy_rejects_free_map_with_unknown_padding():
    """Unknown padded bounds must not let an obstacle-free OMap pass validation."""

    config = BEVConfig()
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[[0, -1], :] = config.unknown

    with pytest.raises(RuntimeError, match="no occupied cells"):
        _validate_external_scene_occupancy(grid, config, "/World/Scene")

    grid[10, 12] = config.occupied
    _validate_external_scene_occupancy(grid, config, "/World/Scene")


def test_isaac_omap_binding_uses_the_45_bindings_module():
    """Importing _omap from the package root fails in the installed Isaac Sim 4.5."""
    imported_names = []
    expected_binding = object()

    def import_module(name):
        imported_names.append(name)
        return expected_binding

    assert _import_omap_binding(import_module) is expected_binding
    assert imported_names == ["isaacsim.asset.gen.omap.bindings._omap"]


def test_robot_collision_traversal_uses_prim_range_and_restores_attributes():
    """Installed OpenUSD Prim has no GetDescendants method; PrimRange owns traversal."""

    class Attribute:
        def __init__(self, enabled):
            self.enabled = enabled

        def IsValid(self):
            return True

        def Get(self):
            return self.enabled

        def Set(self, value):
            self.enabled = value

    class Prim:
        def __init__(self):
            self.attribute = Attribute(True)

        def HasAPI(self, api):
            return True

    class CollisionAPI:
        def __init__(self, prim):
            self.prim = prim

        def GetCollisionEnabledAttr(self):
            return self.prim.attribute

    root, child = Prim(), Prim()

    with _robot_collisions_disabled(root, CollisionAPI, lambda prim: (prim, child)):
        assert not root.attribute.enabled
        assert not child.attribute.enabled

    assert root.attribute.enabled
    assert child.attribute.enabled


def test_bbox_cache_uses_default_time_and_default_purpose():
    """Omitting either constructor argument makes the OpenUSD cache fail at runtime."""

    class FakeTimeCode:
        @staticmethod
        def Default():
            return "default-time"

    class FakeUsd:
        TimeCode = FakeTimeCode

    class FakeTokens:
        default_ = "default-purpose"

    class FakeBBoxCache:
        def __init__(self, time, purposes):
            self.time = time
            self.purposes = purposes

    class FakeUsdGeom:
        BBoxCache = FakeBBoxCache
        Tokens = FakeTokens

    cache = _create_world_bbox_cache(FakeUsdGeom, FakeUsd)

    assert cache.time == "default-time"
    assert cache.purposes == ["default-purpose"]


def test_base_forward_maps_up_and_base_left_maps_image_left():
    cfg = BEVConfig(extent_m=8.0, resolution_m=0.1)
    points = np.array([[1.0, 0.0, 0.2], [0.0, 1.0, 0.2]])
    rows, cols = world_points_to_local_bev(points, np.zeros(3), 0.0, cfg)
    assert (rows[0], cols[0]) == (30, 40)
    assert (rows[1], cols[1]) == (40, 30)


def test_world_forward_rotates_with_base_yaw():
    cfg = BEVConfig(extent_m=8.0, resolution_m=0.1)
    rows, cols = world_points_to_local_bev(
        np.array([[0.0, 1.0, 0.2]]), np.zeros(3), np.pi / 2, cfg
    )
    assert (rows[0], cols[0]) == (30, 40)


def test_crop_samples_global_map_in_local_base_orientation():
    cfg = BEVConfig()
    global_map = np.zeros((160, 160), dtype=np.uint8)
    # Global map axes are world X/Y and its [0, 0] cell center is (-8, -8).
    global_map[90, 80] = cfg.occupied
    local = crop_global_occupancy(global_map, (-8.0, -8.0), (0.0, 0.0), 0.0, cfg)
    assert local.dtype == np.uint8
    assert local[30, 40] == cfg.occupied
    assert local[0, 0] == cfg.free


def test_crop_fills_out_of_bounds_with_unknown():
    cfg = BEVConfig()
    global_map = np.full((3, 3), cfg.free, dtype=np.uint8)
    local = crop_global_occupancy(global_map, (0.0, 0.0), (0.0, 0.0), 0.0, cfg)
    assert local[40, 40] == cfg.free
    assert local[0, 0] == cfg.unknown


def test_clearance_subtracts_ball_radius():
    bev = np.zeros((80, 80), dtype=np.uint8)
    bev[40, 50] = 1
    clearance = obstacle_clearance(bev, ball_radius=0.4, config=BEVConfig())
    assert clearance == pytest.approx(0.6)


def test_clearance_is_infinite_when_no_occupied_cells():
    clearance = obstacle_clearance(np.zeros((80, 80), dtype=np.uint8), 0.4, BEVConfig())
    assert clearance == np.inf


def test_spawn_free_requires_ball_footprint_to_be_free():
    """An occupied cell inside the projected ball makes a spawn invalid."""

    config = BEVConfig()
    grid = np.zeros((80, 80), dtype=np.uint8)
    origin = np.array([-5.0, -5.0])
    # World (0.5, 0.5) is the center of global cell (55, 55).
    grid[55, 55] = config.occupied

    assert is_spawn_free(grid, origin, np.array([0.0, 0.0]), 0.4, config)
    assert not is_spawn_free(grid, origin, np.array([0.5, 0.5]), 0.4, config)
    assert not is_spawn_free(grid, origin, np.array([9.9, 9.9]), 0.4, config)


def test_rasterize_analytic_cylinder_produces_circular_occupied_footprint():
    """Exact Cylinder cross-sections must yield a symmetric circle, not a hull."""

    grid = rasterize_analytic_primitives(
        [
            {
                "kind": "cylinder",
                "center_xy": (0.0, 0.0),
                "radius": 0.5,
                "z_min": 0.0,
                "z_max": 1.0,
            }
        ],
        global_origin_xy=np.array([-5.0, -5.0]),
        grid_shape=(100, 100),
        resolution_m=0.1,
        occupied=1,
        scan_z=0.4,
    )

    assert grid[50, 50] == 1
    assert grid[40, 40] == 0
    occupied = np.argwhere(grid == 1)
    assert len(occupied) > 0
    distances = np.hypot(occupied[:, 0] - 50, occupied[:, 1] - 50) * 0.1
    assert distances.max() <= 0.5 + 1e-9


def test_rasterize_analytic_sphere_slice_radius_scales_with_scan_height():
    """A sphere's horizontal slice shrinks as the scan moves away from its center."""

    sphere = {
        "kind": "sphere",
        "center_xy": (0.0, 0.0),
        "center_z": 1.0,
        "radius": 0.5,
    }

    at_center = rasterize_analytic_primitives(
        [sphere],
        global_origin_xy=np.array([-5.0, -5.0]),
        grid_shape=(100, 100),
        resolution_m=0.1,
        occupied=1,
        scan_z=1.0,
    )
    near_edge = rasterize_analytic_primitives(
        [sphere],
        global_origin_xy=np.array([-5.0, -5.0]),
        grid_shape=(100, 100),
        resolution_m=0.1,
        occupied=1,
        scan_z=1.4,
    )
    outside = rasterize_analytic_primitives(
        [sphere],
        global_origin_xy=np.array([-5.0, -5.0]),
        grid_shape=(100, 100),
        resolution_m=0.1,
        occupied=1,
        scan_z=1.6,
    )

    assert int(at_center.sum()) > int(near_edge.sum()) > 0
    assert int(outside.sum()) == 0


def test_height_band_projection_includes_low_obstacle_missed_by_center_slice():
    """Any obstacle intersecting the ball's vertical body band must be occupied."""

    assert hasattr(ground_truth, "rasterize_analytic_primitives_in_height_band")

    low_cylinder = {
        "kind": "cylinder",
        "center_xy": (0.0, 0.0),
        "radius": 0.3,
        "z_min": 0.05,
        "z_max": 0.20,
    }

    center_slice = rasterize_analytic_primitives(
        [low_cylinder],
        global_origin_xy=np.array([-1.0, -1.0]),
        grid_shape=(20, 20),
        resolution_m=0.1,
        occupied=1,
        scan_z=0.45,
    )
    body_band = ground_truth.rasterize_analytic_primitives_in_height_band(
        [low_cylinder],
        global_origin_xy=np.array([-1.0, -1.0]),
        grid_shape=(20, 20),
        resolution_m=0.1,
        occupied=1,
        lower_z=0.05,
        upper_z=0.85,
    )

    assert center_slice[10, 10] == 0
    assert body_band[10, 10] == 1


def test_height_band_projection_includes_obstacle_touching_upper_part_of_body():
    upper_cylinder = {
        "kind": "cylinder",
        "center_xy": (0.0, 0.0),
        "radius": 0.3,
        "z_min": 0.70,
        "z_max": 1.10,
    }

    body_band = ground_truth.rasterize_analytic_primitives_in_height_band(
        [upper_cylinder],
        global_origin_xy=np.array([-1.0, -1.0]),
        grid_shape=(20, 20),
        resolution_m=0.1,
        occupied=1,
        lower_z=0.05,
        upper_z=0.85,
    )

    assert body_band[10, 10] == 1


def test_height_band_projection_excludes_geometry_above_body():
    above_body = {
        "kind": "cylinder",
        "center_xy": (0.0, 0.0),
        "radius": 0.3,
        "z_min": 0.90,
        "z_max": 1.10,
    }

    body_band = ground_truth.rasterize_analytic_primitives_in_height_band(
        [above_body],
        global_origin_xy=np.array([-1.0, -1.0]),
        grid_shape=(20, 20),
        resolution_m=0.1,
        occupied=1,
        lower_z=0.05,
        upper_z=0.85,
    )

    assert not np.any(body_band)


def test_height_band_projection_uses_largest_in_band_sphere_cross_section():
    sphere = {
        "kind": "sphere",
        "center_xy": (0.0, 0.0),
        "center_z": 1.0,
        "radius": 0.5,
    }

    body_band = ground_truth.rasterize_analytic_primitives_in_height_band(
        [sphere],
        global_origin_xy=np.array([-1.0, -1.0]),
        grid_shape=(20, 20),
        resolution_m=0.1,
        occupied=1,
        lower_z=0.05,
        upper_z=0.85,
    )
    expected_radius = np.sqrt(0.5**2 - 0.15**2)
    occupied = np.argwhere(body_band == 1)
    distances = np.hypot(occupied[:, 0] - 10, occupied[:, 1] - 10) * 0.1

    assert distances.max() <= expected_radius
    assert distances.max() > expected_radius - 0.1


def test_analytic_global_map_projects_full_ball_height(monkeypatch):
    assert "body_radius_m" in inspect.signature(build_analytic_global_map).parameters

    monkeypatch.setattr(
        ground_truth.IsaacOccupancyProvider,
        "_scene_bounds",
        lambda self, usd_geom, usd: (
            np.array([-1.0, -1.0, 0.0]),
            np.array([1.0, 1.0, 1.0]),
        ),
    )
    monkeypatch.setattr(
        ground_truth.IsaacOccupancyProvider,
        "_robot_origin",
        lambda self, usd_geom: np.array([0.0, 0.0, 0.45]),
    )
    monkeypatch.setattr(
        ground_truth,
        "collect_analytic_obstacles",
        lambda *args: [
            {
                "kind": "cylinder",
                "center_xy": (0.0, 0.0),
                "radius": 0.3,
                "z_min": 0.05,
                "z_max": 0.20,
            }
        ],
    )

    occupancy = build_analytic_global_map(
        stage=object(),
        scene_root="/World/Scene",
        robot_root="/World/Robot",
        config=BEVConfig(extent_m=2.0, resolution_m=0.1),
        body_radius_m=0.4,
        usd=object(),
        usd_geom=object(),
        usd_physics=object(),
    )

    assert occupancy.grid[9, 9] == 1


def test_history_mask_moves_backward_when_robot_moves_forward():
    previous = np.zeros((80, 80), dtype=np.uint8)
    previous[30, 40] = 1
    warped = warp_history_mask(previous, (0.0, 0.0, 0.0), (0.5, 0.0, 0.0), BEVConfig())
    assert warped.dtype == np.uint8
    assert warped[35, 40] == 1
    assert warped[30, 40] == 0


def test_history_warp_rounds_subcell_source_coordinates_to_nearest_cell():
    cfg = BEVConfig()
    previous = np.zeros(cfg.shape, dtype=np.uint8)
    # Current row 31 samples previous world x = 0.9 + 0.02 = 0.92 m.
    # The nearest previous cell is row 31 (x=0.9), not row 30 (x=1.0).
    previous[31, 40] = cfg.occupied
    warped = warp_history_mask(previous, (0.0, 0.0, 0.0), (0.02, 0.0, 0.0), cfg)
    assert warped[31, 40] == cfg.occupied


def test_history_warp_fills_newly_exposed_cells_with_unknown():
    previous = np.zeros((80, 80), dtype=np.uint8)
    warped = warp_history_mask(previous, (0.0, 0.0, 0.0), (100.0, 0.0, 0.0), BEVConfig())
    assert np.all(warped == BEVConfig().unknown)


def test_visibility_history_expires_the_sixth_oldest_frame():
    """Recursively warping an aggregate would keep the first observation forever."""
    config = BEVConfig()
    history = VisibilityHistory(max_frames=5, config=config)
    pose = np.zeros(3)

    for frame in range(6):
        current = np.zeros(config.shape, dtype=np.uint8)
        current[10 + frame, 20] = 1
        observed = history.update(current, pose)

    assert observed[10, 20] == 0
    assert np.all(observed[11:16, 20] == 1)


def test_center_depth_marks_forward_ray_visible():
    """Removing ray sampling would lose the cells between base and endpoint."""
    depth = np.array([[2.0]], dtype=np.float32)
    intrinsics = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    camera_to_base = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    mask = depth_to_visible_mask(depth, intrinsics, camera_to_base, BEVConfig())

    assert mask.dtype == np.uint8
    assert mask[40, 40] == 1
    assert mask[20, 40] == 1


def test_invalid_depth_does_not_mark_visible_cells():
    """Treating invalid range readings as rays would fabricate observed space."""
    depth = np.array([[np.inf, np.nan, 0.0]], dtype=np.float32)
    camera_to_base = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    mask = depth_to_visible_mask(depth, np.eye(3), camera_to_base, BEVConfig())

    assert np.count_nonzero(mask) == 0


def test_extract_depth_from_frame_normalizes_to_float32_2d():
    """Returning raw frame values would leak unsupported dtype or shape downstream."""
    result = _extract_depth_from_frame(
        {"distance_to_image_plane": np.array([[1.0, 2.0]], dtype=np.float64)},
        "/World/camera",
    )

    assert result.dtype == np.float32
    np.testing.assert_array_equal(result, np.array([[1.0, 2.0]], dtype=np.float32))


@pytest.mark.parametrize(
    ("frame", "message"),
    [
        ({}, "distance_to_image_plane"),
        ({"distance_to_image_plane": np.array([1.0, 2.0])}, "shape"),
    ],
)
def test_extract_depth_from_frame_rejects_missing_or_nonimage_depth(frame, message):
    """Malformed camera frames must fail rather than emit ambiguous depth labels."""
    with pytest.raises(RuntimeError, match=message):
        _extract_depth_from_frame(frame, "/World/camera")


def test_omap_buffer_conversion_maps_axis_zero_to_world_x():
    """Isaac OMap stores the first (X) axis as the fastest-varying index."""
    raw = np.array([0, 1, 0, 1, 0, 0], dtype=np.float32)

    converted = omap_buffer_to_global_occupancy(raw, dimensions=(2, 3, 1))

    np.testing.assert_array_equal(converted, np.array([[0, 0, 0], [1, 1, 0]], dtype=np.uint8))
    assert converted.dtype == np.uint8


class _HeightPrim:
    def __init__(self, collision_enabled=True):
        self.collision_enabled = collision_enabled

    def IsValid(self):
        return True

    def HasAPI(self, _api):
        return False


class _HeightStage:
    def __init__(self):
        self.robot = _HeightPrim()

    def GetPrimAtPath(self, path):
        return self.robot if path == "/World/Robot" else _HeightPrim()


class _HeightUsd:
    @staticmethod
    def PrimRange(prim):
        return (prim,)


class _HeightPhysics:
    class CollisionAPI:
        pass


def _height_occupancy():
    return GlobalOccupancy(
        grid=np.zeros((2, 2), dtype=np.uint8),
        global_origin_xy=np.array([0.0, 0.0]),
        resolution_m=1.0,
    )


def test_raycast_provider_aggregates_vertical_hit_envelopes_and_keeps_no_hit_zero():
    """Dropping all but one hit would lose a cell's lower or upper envelope."""

    def hit_heights(origin, direction, distance):
        assert tuple(direction) == (0.0, 0.0, -1.0)
        assert distance > 2.0
        by_cell = {
            (0.0, 0.0): (4.0, 1.0),
            (1.0, 1.0): (2.0, 6.0),
        }
        return by_cell.get(tuple(origin[:2]), ())

    provider = IsaacHeightProvider(
        stage=_HeightStage(),
        scene_root="/World/Scene",
        robot_root="/World/Robot",
        occupancy=_height_occupancy(),
        config=BEVConfig(resolution_m=1.0),
        raycast_heights=hit_heights,
    )
    provider._scene_bounds = lambda _geom, _usd: (
        np.array([0.0, 0.0, -1.0]), np.array([2.0, 2.0, 7.0])
    )

    elevation = provider._build_global_map(_HeightUsd, object(), _HeightPhysics, hit_heights)

    assert elevation.valid.dtype == np.uint8
    assert elevation.valid.tolist() == [[1, 0], [0, 1]]
    assert elevation.height_min.tolist() == [[1.0, 0.0], [0.0, 2.0]]
    assert elevation.height_max.tolist() == [[4.0, 0.0], [0.0, 6.0]]


@pytest.mark.parametrize(
    ("scene_minimum_z", "expected_lower_z"),
    [(3.0, 0.0), (-2.0, -2.0)],
)
def test_raycast_provider_bounds_reach_default_ground_without_clipping_lower_terrain(
    scene_minimum_z, expected_lower_z
):
    """A z=0 floor must be raycast even when clutter bounds start above it."""

    expected_maximum_z = 8.0
    resolution = 1.0

    def hit_heights(origin, direction, distance):
        assert tuple(direction) == (0.0, 0.0, -1.0)
        assert origin[2] == pytest.approx(expected_maximum_z + resolution)
        assert distance == pytest.approx(
            expected_maximum_z - expected_lower_z + 2.0 * resolution
        )
        return (0.0,)

    provider = IsaacHeightProvider(
        stage=_HeightStage(),
        scene_root="/World/Scene",
        robot_root="/World/Robot",
        occupancy=_height_occupancy(),
        config=BEVConfig(resolution_m=resolution),
        raycast_heights=hit_heights,
    )
    provider._scene_bounds = lambda _geom, _usd: (
        np.array([0.0, 0.0, scene_minimum_z]),
        np.array([2.0, 2.0, expected_maximum_z]),
    )

    elevation = provider._build_global_map(_HeightUsd, object(), _HeightPhysics, hit_heights)

    assert np.all(elevation.valid == 1)


def test_raycast_provider_syncs_physics_while_robot_collision_is_disabled():
    """Syncing after restoring robot collision would leak the robot into height labels."""

    calls = []
    provider = IsaacHeightProvider(
        stage=_HeightStage(),
        scene_root="/World/Scene",
        robot_root="/World/Robot",
        occupancy=_height_occupancy(),
        config=BEVConfig(resolution_m=1.0),
        physics_sync=lambda: calls.append("sync"),
        raycast_heights=lambda *_args: (),
    )
    provider._scene_bounds = lambda _geom, _usd: (
        np.array([0.0, 0.0, -1.0]), np.array([2.0, 2.0, 1.0])
    )

    provider._build_global_map(_HeightUsd, object(), _HeightPhysics, provider.raycast_heights)

    assert calls == ["sync"]


def test_forced_analytic_elevation_requires_full_free_cell_coverage(monkeypatch):
    """Accepting a partially analytic map would fabricate free-cell height truth."""

    monkeypatch.setattr(
        ground_truth,
        "collect_analytic_obstacles",
        lambda *_args: [{"kind": "cylinder", "center_xy": (0.0, 0.0), "radius": 0.1,
                         "z_min": 0.0, "z_max": 1.0}],
    )

    with pytest.raises(AnalyticElevationCoverageError, match="free occupancy"):
        build_global_elevation(
            "analytic", _HeightStage(), "/World/Scene", "/World/Robot", _height_occupancy(),
            BEVConfig(resolution_m=1.0), object(), object(), object(),
        )


def test_forced_raycast_elevation_uses_provider_and_validates_coverage(monkeypatch):
    """Bypassing the provider or final validation could emit an incomplete teacher."""

    calls = []

    class FakeProvider:
        def __init__(self, **kwargs):
            calls.append(kwargs)

        def build_global_map(self):
            return GlobalElevation(
                np.zeros((2, 2), dtype=np.float32), np.ones((2, 2), dtype=np.float32),
                np.ones((2, 2), dtype=np.uint8), (0.0, 0.0), 1.0,
            )

    monkeypatch.setattr(ground_truth, "IsaacHeightProvider", FakeProvider)

    result = build_global_elevation(
        "raycast", _HeightStage(), "/World/Scene", "/World/Robot", _height_occupancy(),
        BEVConfig(resolution_m=1.0), object(), object(), object(),
    )

    assert len(calls) == 1
    assert isinstance(result, GlobalElevation)


def test_auto_elevation_falls_back_only_for_documented_analytic_errors(monkeypatch):
    """Unexpected analytic failures must remain visible instead of silently changing teachers."""

    class FakeProvider:
        def __init__(self, **_kwargs):
            pass

        def build_global_map(self):
            return GlobalElevation(
                np.zeros((2, 2), dtype=np.float32), np.ones((2, 2), dtype=np.float32),
                np.ones((2, 2), dtype=np.uint8), (0.0, 0.0), 1.0,
            )

    monkeypatch.setattr(ground_truth, "IsaacHeightProvider", FakeProvider)
    monkeypatch.setattr(
        ground_truth, "collect_analytic_obstacles",
        lambda *_args: (_ for _ in ()).throw(AnalyticElevationUnsupportedError("tilted collision")),
    )

    result = build_global_elevation(
        "auto", _HeightStage(), "/World/Scene", "/World/Robot", _height_occupancy(),
        BEVConfig(resolution_m=1.0), object(), object(), object(),
    )
    assert isinstance(result, GlobalElevation)

    monkeypatch.setattr(
        ground_truth, "collect_analytic_obstacles",
        lambda *_args: (_ for _ in ()).throw(RuntimeError("broken stage")),
    )
    with pytest.raises(RuntimeError, match="broken stage"):
        build_global_elevation(
            "auto", _HeightStage(), "/World/Scene", "/World/Robot", _height_occupancy(),
            BEVConfig(resolution_m=1.0), object(), object(), object(),
        )


def test_raycast_elevation_raises_when_provider_cannot_cover_free_cells(monkeypatch):
    """A no-hit generic query cannot be represented as a valid free-cell teacher."""

    class EmptyProvider:
        def __init__(self, **_kwargs):
            pass

        def build_global_map(self):
            return GlobalElevation(
                np.zeros((2, 2), dtype=np.float32), np.zeros((2, 2), dtype=np.float32),
                np.zeros((2, 2), dtype=np.uint8), (0.0, 0.0), 1.0,
            )

    monkeypatch.setattr(ground_truth, "IsaacHeightProvider", EmptyProvider)

    with pytest.raises(ValueError, match="free occupancy"):
        build_global_elevation(
            "raycast", _HeightStage(), "/World/Scene", "/World/Robot", _height_occupancy(),
            BEVConfig(resolution_m=1.0), object(), object(), object(),
        )
