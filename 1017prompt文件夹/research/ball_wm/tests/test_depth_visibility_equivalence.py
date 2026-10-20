"""Exact visibility-mask parity with the pre-optimization scalar ray rasterizer."""

import sys
import importlib.util
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from bev_ground_truth import BEVConfig, depth_to_visible_mask
import bev_ground_truth


@pytest.fixture(autouse=True, params=["numpy", "numba"])
def visibility_backend(request, monkeypatch):
    if request.param == "numpy":
        monkeypatch.setattr(bev_ground_truth, "_numba_visibility_rasterizer", lambda: None)
    elif importlib.util.find_spec("numba") is None:
        pytest.skip("optional Numba accelerator is not installed")
    else:
        assert bev_ground_truth._numba_visibility_rasterizer() is not None


def scalar_visibility_reference(depth, intrinsics, camera_to_base, config):
    """Frozen scalar algorithm, including half-cell samples and endpoint clamping.

    Deliberately does not call production projection or rasterization helpers.
    """

    depths = np.asarray(depth, dtype=np.float64)
    inverse = np.linalg.inv(np.asarray(intrinsics, dtype=np.float64))
    transform = np.asarray(camera_to_base, dtype=np.float64)
    origin = transform[:3, 3]
    mask = np.zeros(config.shape, dtype=np.uint8)
    valid_rows, valid_cols = np.nonzero(np.isfinite(depths) & (depths > 0.0))
    for row, col in zip(valid_rows, valid_cols):
        pixel = np.array([float(col), float(row), 1.0])
        endpoint_camera = inverse @ (pixel * depths[row, col])
        homogeneous = transform @ np.append(endpoint_camera, 1.0)
        if homogeneous[3] == 0.0:
            continue
        endpoint = homogeneous[:3] / homogeneous[3]
        direction = endpoint[:2] - origin[:2]
        length = float(np.linalg.norm(direction))
        if length == 0.0:
            points = origin[:2][None, :]
        else:
            spacing = config.resolution_m / 2.0
            fractions = np.minimum(np.arange(0.0, length + spacing, spacing) / length, 1.0)
            points = origin[:2] + fractions[:, None] * direction
        rows = np.floor(config.center[0] - points[:, 0] / config.resolution_m + 0.5).astype(np.int64)
        cols = np.floor(config.center[1] - points[:, 1] / config.resolution_m + 0.5).astype(np.int64)
        inside = (rows >= 0) & (rows < config.shape[0]) & (cols >= 0) & (cols < config.shape[1])
        mask[rows[inside], cols[inside]] = 1
    return mask


def optical_transform(yaw=0.0, translation=(0.0, 0.0, 0.0)):
    """Optical Z forward, optical X right, with a yawed base-frame camera."""

    c, s = np.cos(yaw), np.sin(yaw)
    yaw_rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    transform = np.eye(4)
    transform[:3, :3] = yaw_rotation @ np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    transform[:3, 3] = translation
    return transform


def assert_reference_equal(depth, intrinsics, transform, config):
    expected = scalar_visibility_reference(depth, intrinsics, transform, config)
    actual = depth_to_visible_mask(depth, intrinsics, transform, config)
    assert actual.dtype == np.uint8
    np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("dtype", [np.float32, np.float64])
def test_visibility_preserves_every_valid_pixel_with_skew_and_camera_rotation(seed, dtype):
    """Dropping pixels, skew, rotation, or translation changes the scalar teacher."""

    rng = np.random.default_rng(seed)
    depth = rng.uniform(0.01, 12.0, size=(17, 23)).astype(dtype)
    depth.flat[::13] = np.nan
    depth.flat[1::17] = np.inf
    depth.flat[2::19] = -1.0
    depth.flat[3::23] = 0.0
    intrinsics = np.array([[21.0, 1.3, 10.2], [0.2, 19.0, 7.7], [0.0, 0.0, 1.0]])
    transform = optical_transform(seed * 0.73, (seed * 0.31 - 1.0, 0.17, 0.22))
    pitch = 0.19 * seed
    pitch_rotation = np.array([[np.cos(pitch), 0.0, np.sin(pitch)], [0.0, 1.0, 0.0], [-np.sin(pitch), 0.0, np.cos(pitch)]])
    transform[:3, :3] = pitch_rotation @ transform[:3, :3]
    assert_reference_equal(depth[:, ::-1], intrinsics, transform, BEVConfig(extent_m=4.2, resolution_m=0.07))


@pytest.mark.parametrize("endpoint", [0.05, 0.15, 0.35, 3.95, 4.05, 4.15, 17.0])
@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("neighbor", [-1, 0, 1])
def test_visibility_preserves_half_cell_and_grid_edge_rounding(endpoint, direction, neighbor):
    """Half-cell ties and their immediate float neighbors retain identical cells."""

    if neighbor:
        endpoint = np.nextafter(endpoint, np.inf if neighbor > 0 else -np.inf)
    transform = np.eye(4)
    transform[0, 2] = direction
    transform[1, 3] = 0.05
    assert_reference_equal([[endpoint]], np.eye(3), transform, BEVConfig())


@pytest.mark.parametrize("translation", [(-8.0, 0.15, 0.0), (8.0, -4.05, 0.0), (0.0, 0.0, 0.0)])
def test_visibility_preserves_long_rays_starting_outside_the_grid(translation):
    depth = np.array([[0.02, 2.1, 10.0, 27.0], [35.0, 0.1, np.inf, 12.0]])
    assert_reference_equal(depth, np.array([[2.0, 0.1, 2.0], [0.0, 2.0, 0.5], [0.0, 0.0, 1.0]]), optical_transform(0.38, translation), BEVConfig())


def test_visibility_preserves_projective_division_and_skips_zero_homogeneous_weight():
    transform = optical_transform(0.2, (0.13, -0.21, 0.1))
    transform[3] = [0.0, 0.0, 1.0, -2.0]
    assert_reference_equal([[2.0, 1.0, 3.0], [0.1, 5.0, 2.0]], np.eye(3), transform, BEVConfig())


def test_visibility_preserves_zero_length_rays_and_invalid_depth():
    assert_reference_equal([[1.0]], np.eye(3), np.eye(4), BEVConfig())
    assert_reference_equal([[np.nan, np.inf, -np.inf, 0.0, -1.0]], np.eye(3), np.eye(4), BEVConfig())
    assert_reference_equal(np.zeros((0, 3)), np.eye(3), np.eye(4), BEVConfig())


def test_visibility_keeps_isolated_valid_pixels_off_regular_sampling_strides():
    depth = np.zeros((13, 19))
    depth[1, 17] = 3.13
    depth[7, 5] = 4.27
    depth[11, 13] = 2.19
    assert_reference_equal(depth, np.array([[15.0, 0.0, 9.0], [0.0, 15.0, 6.0], [0.0, 0.0, 1.0]]), optical_transform(), BEVConfig())


def test_visibility_preserves_masks_across_ray_batch_boundaries():
    depth = np.random.default_rng(23).uniform(0.02, 1.8, size=(65, 67))
    assert_reference_equal(depth, np.array([[70.0, 0.0, 33.0], [0.0, 65.0, 32.0], [0.0, 0.0, 1.0]]), optical_transform(-0.8, (0.27, -0.45, 0.2)), BEVConfig())


@pytest.mark.parametrize("depth", [1e17, 1e30, 1e200])
def test_visibility_rejects_unrepresentable_rays_like_the_scalar_reference(depth):
    transform = optical_transform()
    with np.errstate(over="ignore", invalid="ignore"):
        with pytest.raises(ValueError):
            scalar_visibility_reference([[depth]], np.eye(3), transform, BEVConfig())
        with pytest.raises(ValueError):
            depth_to_visible_mask([[depth]], np.eye(3), transform, BEVConfig())


def benchmark_visibility():
    """Run directly to compare exact masks and cold/warm 640x480 timings."""

    import json
    import time

    depth = np.random.default_rng(20260922).uniform(0.1, 8.0, size=(480, 640)).astype(np.float32)
    intrinsics = np.array([[584.8888889, 0.0, 319.5], [0.0, 584.8888889, 239.5], [0.0, 0.0, 1.0]])
    transform = optical_transform(0.07, (0.08, 0.04, 0.28))
    config = BEVConfig()
    started = time.perf_counter()
    reference = scalar_visibility_reference(depth, intrinsics, transform, config)
    report = {
        "input": "synthetic uniform depth in [0.1,8.0] metres",
        "shape": list(depth.shape),
        "valid_pixels": depth.size,
        "numpy_version": np.__version__,
        "scalar_seconds": time.perf_counter() - started,
    }

    def timed():
        started = time.perf_counter()
        actual = depth_to_visible_mask(depth, intrinsics, transform, config)
        elapsed = time.perf_counter() - started
        np.testing.assert_array_equal(actual, reference)
        return elapsed

    accelerator_factory = bev_ground_truth._numba_visibility_rasterizer
    try:
        bev_ground_truth._numba_visibility_rasterizer = lambda: None
        report["numpy_seconds"] = [timed() for _ in range(5)]
    finally:
        bev_ground_truth._numba_visibility_rasterizer = accelerator_factory
    if importlib.util.find_spec("numba") is not None:
        report["numba_first_call_seconds"] = timed()
        report["numba_warm_seconds"] = [timed() for _ in range(5)]
        import numba
        import llvmlite

        report["numba_version"] = numba.__version__
        report["llvmlite_version"] = llvmlite.__version__
        rasterizer = accelerator_factory()
        assert rasterizer is not None and rasterizer.nopython_signatures
        report["numba_nopython_signatures"] = [str(signature) for signature in rasterizer.nopython_signatures]
    report["all_masks_exact_equal"] = True
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    benchmark_visibility()
