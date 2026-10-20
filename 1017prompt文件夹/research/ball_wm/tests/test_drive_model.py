"""Tests for the versioned, direction-aware ball drive model."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from benchmark.drive_model import DriveModel, SteeringSample


PROFILE_PATH = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "drive_dynamics_v1.json"
)


def test_drive_model_preserves_measured_forward_and_reverse_response():
    model = DriveModel.from_json(PROFILE_PATH)

    assert model.curvature(1.0, math.radians(-20.0)) == pytest.approx(
        0.33146889578522615
    )
    assert model.curvature(-1.0, math.radians(-20.0)) == pytest.approx(
        0.4087968778382846
    )
    assert model.curvature(1.0, 0.0) == pytest.approx(-0.060383292320773566)
    assert model.curvature(-1.0, 0.0) == pytest.approx(-0.05682728334598315)


def test_drive_model_loads_velocity_limited_v2_profile_metadata():
    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["schema_version"] = "drive_dynamics_v2"
    payload["controller"]["auxiliary_velocity_limit_rad_s"] = 0.30

    model = DriveModel.from_mapping(payload)

    assert model.auxiliary_velocity_limit_rad_s == pytest.approx(0.30)


def test_drive_model_interpolates_without_mixing_drive_directions():
    model = DriveModel.from_json(PROFILE_PATH)

    forward = model.curvature(1.0, math.radians(-15.0))
    reverse = model.curvature(-1.0, math.radians(-15.0))

    assert 0.1577 < forward < 0.3315
    assert 0.1988 < reverse < 0.4088
    assert forward != pytest.approx(reverse)


def test_speed_is_zero_at_zero_main_and_continuous_between_calibrated_commands():
    model = DriveModel.from_json(PROFILE_PATH)
    assert model.speed(0.0, 0.0) == 0.0
    midpoint = model.speed(0.75, 0.0)
    assert model.speed(0.5, 0.0) < midpoint < model.speed(1.0, 0.0)
    assert model.speed(-0.75, 0.0) > 0.0


def test_below_domain_0p35_query_uses_mixed_documented_endpoint_behavior():
    model = DriveModel.from_json(PROFILE_PATH)
    auxiliary = math.radians(20.0)

    assert model.curvature(0.35, auxiliary) == pytest.approx(
        model.curvature(0.5, auxiliary)
    )
    assert model.auxiliary_for_curvature(
        0.35, model.curvature(0.5, auxiliary)
    ) == pytest.approx(auxiliary)
    assert model.speed(0.35, auxiliary) == pytest.approx(
        0.7 * model.speed(0.5, auxiliary)
    )


def test_repeated_speed_and_curvature_queries_reuse_directional_interpolation_tables(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model = DriveModel.from_json(PROFILE_PATH)
    tables_by_direction: dict[
        str, list[tuple[tuple[SteeringSample, ...], ...]]
    ] = {
        "forward": [],
        "reverse": [],
    }
    original_steering_groups = DriveModel._steering_groups

    def record_steering_groups(
        self: DriveModel, main_command_rad_s: float
    ) -> tuple[tuple[float, ...], tuple[tuple[SteeringSample, ...], ...]]:
        magnitudes, groups = original_steering_groups(self, main_command_rad_s)
        direction = "forward" if main_command_rad_s > 0.0 else "reverse"
        tables_by_direction[direction].append(groups)
        return magnitudes, groups

    monkeypatch.setattr(DriveModel, "_steering_groups", record_steering_groups)

    assert model.speed(0.75, math.radians(-15.0)) == pytest.approx(
        0.3042279856176614
    )
    assert model.curvature(0.75, math.radians(-15.0)) == pytest.approx(
        0.3272194925619067
    )
    assert model.speed(-0.75, math.radians(15.0)) == pytest.approx(
        0.29024427686065457
    )
    assert model.curvature(-0.75, math.radians(15.0)) == pytest.approx(
        -0.547875145100237
    )
    assert len(tables_by_direction["forward"]) == 1
    assert len(tables_by_direction["reverse"]) == 1
    assert tables_by_direction["forward"][0] is not tables_by_direction["reverse"][0]


def test_exact_response_cache_reuses_one_interpolation_across_speed_and_curvature(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A repeated valid pair must not re-run either response interpolation."""
    model = DriveModel.from_json(PROFILE_PATH)
    interpolation_calls = 0
    original_interp = np.interp

    def count_interp(*args, **kwargs):
        nonlocal interpolation_calls
        interpolation_calls += 1
        return original_interp(*args, **kwargs)

    monkeypatch.setattr(np, "interp", count_interp)

    for _ in range(2):
        assert model.speed(0.75, math.radians(-15.0)) == pytest.approx(
            0.3042279856176614
        )
        assert model.curvature(0.75, math.radians(-15.0)) == pytest.approx(
            0.3272194925619067
        )

    # One combined response calculation contains four speed and four curvature
    # interpolations.  Repeating either public query must be a cache hit.
    assert interpolation_calls == 8


def test_exact_response_cache_is_not_shared_between_drive_model_instances(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """One model's memoized response must not populate another model's cache."""
    first = DriveModel.from_json(PROFILE_PATH)
    second = DriveModel.from_json(PROFILE_PATH)
    interpolation_calls = 0
    original_interp = np.interp

    def count_interp(*args, **kwargs):
        nonlocal interpolation_calls
        interpolation_calls += 1
        return original_interp(*args, **kwargs)

    monkeypatch.setattr(np, "interp", count_interp)

    assert first.speed(0.75, math.radians(-15.0)) == pytest.approx(
        0.3042279856176614
    )
    assert len(first._response_cache) == 1
    assert len(second._response_cache) == 0
    assert first._response_cache is not second._response_cache

    assert second.speed(0.75, math.radians(-15.0)) == pytest.approx(
        0.3042279856176614
    )
    assert interpolation_calls == 16


def test_exact_response_cache_preserves_zero_and_invalid_query_behavior() -> None:
    """Invalid inputs and the stopped response are validated before memoization."""
    model = DriveModel.from_json(PROFILE_PATH)

    assert model.speed(0.0, 0.0) == 0.0
    assert len(model._response_cache) == 0
    with pytest.raises(ValueError, match="finite"):
        model.speed(0.0, math.nan)
    with pytest.raises(ValueError, match="finite"):
        model.curvature(math.nan, 0.0)
    with pytest.raises(ValueError, match="non-zero"):
        model.curvature(0.0, 0.0)
    assert len(model._response_cache) == 0


def test_exact_response_cache_is_bounded_and_eviction_preserves_response(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Evicted exact inputs recompute the same response without exceeding capacity."""
    model = DriveModel.from_json(PROFILE_PATH)
    interpolation_calls = 0
    original_interp = np.interp

    def count_interp(*args, **kwargs):
        nonlocal interpolation_calls
        interpolation_calls += 1
        return original_interp(*args, **kwargs)

    monkeypatch.setattr(np, "interp", count_interp)

    expected_speed = model.speed(0.5, 0.0)
    for index in range(1, 16_385):
        model.speed(0.5 + index, 0.0)

    assert len(model._response_cache) == 16_384
    assert (0.5, 0.0) not in model._response_cache
    calls_before_requery = interpolation_calls
    assert model.speed(0.5, 0.0) == expected_speed
    assert interpolation_calls == calls_before_requery + 8
    assert len(model._response_cache) == 16_384


def test_drive_model_inverts_curvature_and_saturates_at_measured_aux_limits():
    model = DriveModel.from_json(PROFILE_PATH)

    measured_curvature = model.curvature(1.0, math.radians(-20.0))

    assert model.auxiliary_for_curvature(
        1.0, measured_curvature
    ) == pytest.approx(math.radians(-20.0))
    assert model.auxiliary_for_curvature(1.0, 100.0) == pytest.approx(
        math.radians(-30.0)
    )
    assert model.auxiliary_for_curvature(1.0, -100.0) == pytest.approx(
        math.radians(30.0)
    )


def test_drive_model_interpolates_and_extrapolates_directional_braking():
    model = DriveModel.from_json(PROFILE_PATH)

    assert model.stopping_distance(0.39040078249994153, "forward") == pytest.approx(
        0.5165229634564943
    )
    assert model.stopping_distance(0.398989762118167, "reverse") == pytest.approx(
        0.5712541508775116
    )
    assert model.stopping_distance(0.0, "forward") == pytest.approx(0.0)
    assert model.stopping_distance(0.7, "forward") > 1.0881320208656964
    assert model.stopping_distance(0.7, "reverse") > 1.043870513750411


@pytest.mark.parametrize(
    ("method", "args", "message"),
    [
        ("curvature", (0.0, 0.0), "non-zero"),
        ("curvature", (math.nan, 0.0), "finite"),
        ("auxiliary_for_curvature", (1.0, math.inf), "finite"),
        ("stopping_distance", (-0.1, "forward"), "non-negative"),
        ("stopping_distance", (0.1, "sideways"), "drive_direction"),
    ],
)
def test_drive_model_rejects_invalid_queries(method: str, args: tuple, message: str):
    model = DriveModel.from_json(PROFILE_PATH)

    with pytest.raises(ValueError, match=message):
        getattr(model, method)(*args)


def test_drive_model_rejects_duplicate_or_incomplete_profiles(tmp_path: Path):
    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["steering_samples"].append(dict(payload["steering_samples"][0]))
    duplicate_path = tmp_path / "duplicate.json"
    duplicate_path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate"):
        DriveModel.from_json(duplicate_path)

    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["braking_samples"] = [
        row
        for row in payload["braking_samples"]
        if row["drive_direction"] == "forward"
    ]
    incomplete_path = tmp_path / "incomplete.json"
    incomplete_path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="both drive directions"):
        DriveModel.from_json(incomplete_path)


def test_drive_model_rejects_incompatible_profile_metadata(tmp_path: Path):
    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["controller"]["auxiliary_kp"] = 200.0
    path = tmp_path / "wrong-controller.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="controller metadata"):
        DriveModel.from_json(path)
