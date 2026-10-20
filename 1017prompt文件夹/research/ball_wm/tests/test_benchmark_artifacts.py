"""Tests for atomic, auditable Benchmark V1 episode bundles."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest

from benchmark.manifest import BenchmarkEpisode, HumanReview
from benchmark.artifacts import (
    REQUIRED_AUDIT_FILES,
    REQUIRED_TRAJECTORY_LAYERS,
    AuditBundleWriter,
    artifact_sha256,
    review_episode_bundle,
    write_rgb_video,
)


EXPECTED_FILES = {
    "manifest.json",
    "environment.json",
    "task_card.png",
    "third_person.mp4",
    "stereo_view.mp4",
    "trajectory_overlay.png",
    "actions.npz",
    "states.npz",
    "metrics.json",
    "human_review.json",
}


def _execution_audit_arrays(
    *, state_count: int = 3, action_count: int = 2
) -> dict[str, np.ndarray]:
    return {
        "yaw_world_rad": np.zeros(state_count, dtype=np.float64),
        "linear_velocity_world_m_s": np.zeros((state_count, 3), dtype=np.float32),
        "angular_velocity_world_rad_s": np.zeros((state_count, 3), dtype=np.float32),
        "joint_position_rad": np.zeros((state_count, 3), dtype=np.float32),
        "joint_velocity_rad_s": np.zeros((state_count, 3), dtype=np.float32),
        "applied_action": np.zeros((state_count, 2), dtype=np.float32),
        "policy_reference_index": np.zeros(action_count, dtype=np.int64),
        "policy_cross_track_error_m": np.zeros(action_count, dtype=np.float64),
        "policy_active_drive_direction": np.ones(action_count, dtype=np.int8),
        "policy_desired_drive_direction": np.ones(action_count, dtype=np.int8),
        "policy_pending_drive_direction": np.zeros(action_count, dtype=np.int8),
        "policy_stopped_cycles": np.zeros(action_count, dtype=np.int64),
        "policy_braking_state_code": np.ones(action_count, dtype=np.int8),
    }


def _dynamic_reference_arrays(
    *, reference_count: int = 3, action_count: int = 2
) -> dict[str, np.ndarray]:
    return {
        "dynamic_reference_time_s": np.asarray((0.0, 0.5, 1.0), dtype=np.float64)[
            :reference_count
        ],
        "dynamic_reference_pose": np.asarray(
            ((0.0, 0.0, 0.0), (0.2, 0.0, 0.1), (0.3, 0.0, 0.1)),
            dtype=np.float64,
        )[:reference_count],
        "dynamic_reference_speed_m_s": np.asarray((0.0, 0.3, 0.0), dtype=np.float64)[
            :reference_count
        ],
        "dynamic_reference_command": np.asarray(
            ((0.0, 0.0), (0.25, 0.1), (0.0, 0.0)), dtype=np.float64
        )[:reference_count],
        "dynamic_reference_drive_direction": np.asarray(
            (0, 1, 0), dtype=np.int8
        )[:reference_count],
        "dynamic_reference_index": np.arange(action_count, dtype=np.int64),
        "dynamic_position_error_m": np.zeros(action_count, dtype=np.float64),
        "dynamic_yaw_error_rad": np.zeros(action_count, dtype=np.float64),
        "dynamic_auxiliary_error_rad": np.zeros(action_count, dtype=np.float64),
    }


def _dynamic_certification() -> dict[str, object]:
    return {
        "planning_status": "success",
        "expanded_nodes": 12,
        "direction_changes": 0,
        "minimum_clearance_m": 0.2,
        "maximum_cross_track_error_m": 0.1,
        "mean_cross_track_error_m": 0.05,
        "failure_subreason": None,
        "planner_version": "dynamic_lattice_v2",
        "planning_wall_time_s": 1.25,
        "predicted_duration_s": 1.0,
        "maximum_main_rate_rad_s2": 0.5,
        "maximum_auxiliary_rate_rad_s": 0.2,
    }


def _populate_dynamic_bundle(writer: AuditBundleWriter) -> None:
    _populate_complete_bundle(writer)
    writer.write_json(
        "environment.json",
        {
            "policy": "oracle_v2",
            "privileged": True,
            "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
            "dynamic_planner": {
                "planner_version": "dynamic_lattice_v2",
                "drive_profile_path": "/fixture/drive.json",
                "primitive_duration_s": 1.0,
                "integration_dt_s": 0.02,
                "main_acceleration_limit_rad_s2": 0.5,
                "auxiliary_rate_limit_rad_s": np.deg2rad(30.0),
                "maximum_predicted_duration_s": 32.0,
                "node_budget": 500_000,
                "planning_timeout_s": 60.0,
            },
        },
    )
    writer.write_json("metrics.json", {"certification": _dynamic_certification()})
    writer.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        **_execution_audit_arrays(),
        **_dynamic_reference_arrays(),
    )


def _populate_complete_bundle(writer: AuditBundleWriter) -> None:
    writer.begin()
    writer.write_json("manifest.json", {"schema_version": "fixture"})
    writer.write_json(
        "environment.json",
        {
            "localization_source": "perfect_localization",
            "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
        },
    )
    writer.write_json(
        "metrics.json",
        {"termination_reason": "success", "success": 1.0, "spl": 1.0},
    )
    writer.write_json(
        "human_review.json",
        {"status": "pending", "reviewer": "", "labels": [], "notes": ""},
    )
    writer.write_npz("actions.npz", action=np.zeros((2, 10), dtype=np.float32))
    writer.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        clearance_m=np.ones(3, dtype=np.float32),
    )
    for name in ("task_card.png", "trajectory_overlay.png"):
        Image.new("RGB", (1200, 900), (240, 240, 240)).save(writer.path(name))
    writer.path("third_person.mp4").write_bytes(b"non-empty-third-person-video")
    writer.path("stereo_view.mp4").write_bytes(b"non-empty-stereo-video")


def test_required_bundle_contract_is_exact():
    assert REQUIRED_AUDIT_FILES == EXPECTED_FILES


def test_incomplete_bundle_cannot_publish_and_partial_is_preserved(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e0_0001")
    writer.begin()
    writer.write_json("metrics.json", {})

    with pytest.raises(ValueError, match="missing audit files"):
        writer.publish()

    assert not writer.output_dir.exists()
    assert writer.partial_dir.is_dir()


def test_complete_bundle_publishes_atomically_without_overwrite(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e0_0001")
    _populate_complete_bundle(writer)

    published = writer.publish()

    assert published == writer.output_dir
    assert published.is_dir()
    assert not writer.partial_dir.exists()
    assert EXPECTED_FILES.issubset(path.name for path in published.iterdir())
    with pytest.raises(FileExistsError):
        AuditBundleWriter(tmp_path, "golden_e0_0001").begin()


def test_nonfinite_npz_and_json_are_rejected_without_losing_partial(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e0_0001")
    _populate_complete_bundle(writer)
    writer.write_npz("actions.npz", action=np.asarray([[float("nan")] * 10]))

    with pytest.raises(ValueError, match="finite"):
        writer.publish()
    assert writer.partial_dir.is_dir()

    writer.write_npz("actions.npz", action=np.zeros((1, 10), dtype=np.float32))
    writer.path("metrics.json").write_text('{"spl": NaN}', encoding="utf-8")
    with pytest.raises(ValueError, match="finite"):
        writer.publish()


def test_empty_video_and_missing_trajectory_layers_are_rejected(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e0_0001")
    _populate_complete_bundle(writer)
    writer.path("third_person.mp4").write_bytes(b"")

    with pytest.raises(ValueError, match="video"):
        writer.publish()

    writer.path("third_person.mp4").write_bytes(b"video")
    writer.write_json(
        "environment.json",
        {"trajectory_overlay_layers": ["obstacles", "actual_path"]},
    )
    with pytest.raises(ValueError, match="trajectory.*layers"):
        writer.publish()


def test_privileged_bundle_requires_numeric_kinodynamic_reference_and_certification(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e1_0001")
    _populate_complete_bundle(writer)
    writer.write_json(
        "environment.json",
        {
            "privileged": True,
            "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
        },
    )
    certification = {
        "planning_status": "success",
        "expanded_nodes": 12,
        "direction_changes": 1,
        "minimum_clearance_m": 0.2,
        "maximum_cross_track_error_m": 0.1,
        "mean_cross_track_error_m": 0.05,
        "failure_subreason": None,
    }
    writer.write_json("metrics.json", {"certification": certification})

    with pytest.raises(ValueError, match="kinodynamic_reference"):
        writer.publish()

    writer.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        kinodynamic_reference_pose=np.asarray(
            ((0.0, 0.0, 0.0), (0.3, 0.0, 0.0)), dtype=np.float64
        ),
        kinodynamic_reference_drive_direction=np.asarray((1, -1), dtype=np.int8),
        kinodynamic_reference_command=np.asarray(
            ((1.0, 0.0), (-1.0, 0.1)), dtype=np.float64
        ),
    )
    writer.write_json(
        "metrics.json",
        {
            "termination_reason": "success",
            "success": 1.0,
            "spl": 1.0,
            "certification": certification,
        },
    )

    with pytest.raises(ValueError, match="execution audit"):
        writer.publish()

    writer.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        kinodynamic_reference_pose=np.asarray(
            ((0.0, 0.0, 0.0), (0.3, 0.0, 0.0)), dtype=np.float64
        ),
        kinodynamic_reference_drive_direction=np.asarray((1, -1), dtype=np.int8),
        kinodynamic_reference_command=np.asarray(
            ((1.0, 0.0), (-1.0, 0.1)), dtype=np.float64
        ),
        **_execution_audit_arrays(),
    )
    assert writer.publish().is_dir()


def test_privileged_bundle_requires_complete_certification_metrics(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e1_0003")
    _populate_complete_bundle(writer)
    writer.write_json(
        "environment.json",
        {
            "privileged": True,
            "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
        },
    )
    with pytest.raises(ValueError, match="certification"):
        writer.publish()


def test_privileged_reference_rejects_mismatched_arrays(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e1_0002")
    _populate_complete_bundle(writer)
    writer.write_json(
        "environment.json",
        {
            "privileged": True,
            "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
        },
    )
    writer.write_npz(
        "states.npz",
        kinodynamic_reference_pose=np.zeros((2, 3), dtype=np.float64),
        kinodynamic_reference_drive_direction=np.ones(1, dtype=np.int8),
        kinodynamic_reference_command=np.zeros((2, 2), dtype=np.float64),
    )
    writer.write_json(
        "metrics.json",
        {
            "certification": {
                "planning_status": "success",
                "expanded_nodes": 1,
                "direction_changes": 0,
                "minimum_clearance_m": 0.2,
                "maximum_cross_track_error_m": 0.1,
                "mean_cross_track_error_m": 0.05,
                "failure_subreason": None,
            }
        },
    )

    with pytest.raises(ValueError, match="same length"):
        writer.publish()


def test_dynamic_reference_arrays_are_written_with_auditable_shapes_and_codes():
    from benchmark.dynamic_lattice_planning import DynamicPlanResult, DynamicWaypoint
    from scripts.run_pointgoal_benchmark import (
        _dynamic_reference_arrays as serialize_dynamic_reference,
    )

    plan = DynamicPlanResult(
        status="success",
        waypoints=(
            DynamicWaypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped"),
            DynamicWaypoint(0.2, 0.1, 0.2, 0.25, 0.1, 0.3, 0.5, "forward"),
            DynamicWaypoint(0.3, 0.1, 0.2, 0.0, 0.0, 0.0, 1.0, "stopped"),
        ),
        duration_s=1.0,
    )
    runtime = SimpleNamespace(dynamic_plan=lambda _episode: plan)

    arrays = serialize_dynamic_reference(runtime, SimpleNamespace())

    assert set(arrays) == {
        "dynamic_reference_time_s",
        "dynamic_reference_pose",
        "dynamic_reference_speed_m_s",
        "dynamic_reference_command",
        "dynamic_reference_drive_direction",
    }
    assert arrays["dynamic_reference_time_s"].shape == (3,)
    assert arrays["dynamic_reference_pose"].shape == (3, 3)
    assert arrays["dynamic_reference_speed_m_s"].shape == (3,)
    assert arrays["dynamic_reference_command"].shape == (3, 2)
    np.testing.assert_array_equal(
        arrays["dynamic_reference_drive_direction"], (0, 1, 0)
    )


def test_dynamic_bundle_publishes_only_with_complete_monotonic_aligned_reference(
    tmp_path: Path,
):
    writer = AuditBundleWriter(tmp_path, "dynamic_valid")
    _populate_dynamic_bundle(writer)
    assert writer.publish().is_dir()

    missing = AuditBundleWriter(tmp_path, "dynamic_missing")
    _populate_dynamic_bundle(missing)
    arrays = _dynamic_reference_arrays()
    arrays.pop("dynamic_reference_speed_m_s")
    missing.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        **_execution_audit_arrays(),
        **arrays,
    )
    with pytest.raises(ValueError, match="dynamic_reference.*missing"):
        missing.publish()

    nonmonotonic = AuditBundleWriter(tmp_path, "dynamic_nonmonotonic")
    _populate_dynamic_bundle(nonmonotonic)
    arrays = _dynamic_reference_arrays()
    arrays["dynamic_reference_time_s"] = np.asarray((0.0, 0.5, 0.5))
    nonmonotonic.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        **_execution_audit_arrays(),
        **arrays,
    )
    with pytest.raises(ValueError, match="strictly increasing"):
        nonmonotonic.publish()

    invalid_direction = AuditBundleWriter(tmp_path, "dynamic_direction")
    _populate_dynamic_bundle(invalid_direction)
    arrays = _dynamic_reference_arrays()
    arrays["dynamic_reference_drive_direction"] = np.asarray((0, 2, 0))
    invalid_direction.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        **_execution_audit_arrays(),
        **arrays,
    )
    with pytest.raises(ValueError, match="-1, 0, or 1"):
        invalid_direction.publish()

    misaligned = AuditBundleWriter(tmp_path, "dynamic_diagnostics")
    _populate_dynamic_bundle(misaligned)
    arrays = _dynamic_reference_arrays(action_count=1)
    misaligned.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        **_execution_audit_arrays(),
        **arrays,
    )
    with pytest.raises(ValueError, match="dynamic.*diagnostic"):
        misaligned.publish()


def test_dynamic_bundle_requires_v2_certification_and_frozen_reference_rates(
    tmp_path: Path,
):
    missing = AuditBundleWriter(tmp_path, "dynamic_certification_missing")
    _populate_dynamic_bundle(missing)
    certification = _dynamic_certification()
    certification.pop("planning_wall_time_s")
    missing.write_json("metrics.json", {"certification": certification})
    with pytest.raises(ValueError, match="certification.*planning_wall_time_s"):
        missing.publish()

    main_rate = AuditBundleWriter(tmp_path, "dynamic_main_rate")
    _populate_dynamic_bundle(main_rate)
    arrays = _dynamic_reference_arrays()
    arrays["dynamic_reference_command"] = np.asarray(
        ((0.0, 0.0), (0.6, 0.0), (0.0, 0.0)), dtype=np.float64
    )
    main_rate.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        **_execution_audit_arrays(),
        **arrays,
    )
    with pytest.raises(ValueError, match="main.*rate"):
        main_rate.publish()

    auxiliary_rate = AuditBundleWriter(tmp_path, "dynamic_auxiliary_rate")
    _populate_dynamic_bundle(auxiliary_rate)
    arrays = _dynamic_reference_arrays()
    arrays["dynamic_reference_command"] = np.asarray(
        ((0.0, 0.0), (0.0, 0.6), (0.0, 0.0)), dtype=np.float64
    )
    auxiliary_rate.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        **_execution_audit_arrays(),
        **arrays,
    )
    with pytest.raises(ValueError, match="auxiliary.*rate"):
        auxiliary_rate.publish()


def test_dynamic_bundle_rejects_unprivileged_or_unfrozen_model_metadata(tmp_path: Path):
    unprivileged = AuditBundleWriter(tmp_path, "dynamic_unprivileged")
    _populate_dynamic_bundle(unprivileged)
    environment = {
        "policy": "oracle_v2",
        "privileged": False,
        "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
        "dynamic_planner": {
            "planner_version": "dynamic_lattice_v2",
            "drive_profile_path": "/fixture/drive.json",
            "primitive_duration_s": 1.0,
            "integration_dt_s": 0.02,
            "main_acceleration_limit_rad_s2": 0.5,
            "auxiliary_rate_limit_rad_s": np.deg2rad(30.0),
            "maximum_predicted_duration_s": 32.0,
            "node_budget": 500_000,
            "planning_timeout_s": 60.0,
        },
    }
    unprivileged.write_json("environment.json", environment)
    with pytest.raises(ValueError, match="oracle_v2.*privileged"):
        unprivileged.publish()

    unfrozen = AuditBundleWriter(tmp_path, "dynamic_unfrozen")
    _populate_dynamic_bundle(unfrozen)
    environment["privileged"] = True
    environment["dynamic_planner"]["main_acceleration_limit_rad_s2"] = 0.6
    unfrozen.write_json("environment.json", environment)
    with pytest.raises(ValueError, match="dynamic_planner.*frozen"):
        unfrozen.publish()


def test_dynamic_bundle_rejects_predicted_duration_that_disagrees_with_reference(
    tmp_path: Path,
):
    writer = AuditBundleWriter(tmp_path, "dynamic_duration_mismatch")
    _populate_dynamic_bundle(writer)
    certification = _dynamic_certification()
    certification["predicted_duration_s"] = 0.9
    writer.write_json("metrics.json", {"certification": certification})

    with pytest.raises(ValueError, match="predicted_duration_s.*reference"):
        writer.publish()


def test_v1_privileged_bundle_is_not_forced_to_use_dynamic_v2_schema(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "oracle_v1")
    _populate_complete_bundle(writer)
    writer.write_json(
        "environment.json",
        {
            "policy": "oracle",
            "privileged": True,
            "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
        },
    )
    certification = {
        "planning_status": "success",
        "expanded_nodes": 1,
        "direction_changes": 0,
        "minimum_clearance_m": 0.2,
        "maximum_cross_track_error_m": 0.1,
        "mean_cross_track_error_m": 0.05,
        "failure_subreason": None,
    }
    writer.write_json("metrics.json", {"certification": certification})
    writer.write_npz(
        "states.npz",
        position_world_xy=np.zeros((3, 2), dtype=np.float64),
        kinodynamic_reference_pose=np.asarray(
            ((0.0, 0.0, 0.0), (0.3, 0.0, 0.0)), dtype=np.float64
        ),
        kinodynamic_reference_drive_direction=np.asarray((1, 1), dtype=np.int8),
        kinodynamic_reference_command=np.asarray(
            ((0.5, 0.0), (0.0, 0.0)), dtype=np.float64
        ),
        **_execution_audit_arrays(),
    )

    assert writer.publish().is_dir()


def test_behavior_review_changes_only_human_review_file(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e0_0001")
    _populate_complete_bundle(writer)
    episode_dir = writer.publish()
    protected = {
        name: artifact_sha256(episode_dir / name)
        for name in EXPECTED_FILES - {"human_review.json"}
    }

    result = review_episode_bundle(
        episode_dir,
        status="approved",
        reviewer="project_owner",
        labels=("goal_response", "active_avoidance"),
        notes="GUI and videos checked",
    )

    assert result == episode_dir / "human_review.json"
    review = json.loads(result.read_text(encoding="utf-8"))
    assert review == {
        "status": "approved",
        "reviewer": "project_owner",
        "labels": ["goal_response", "active_avoidance"],
        "notes": "GUI and videos checked",
    }
    assert protected == {
        name: artifact_sha256(episode_dir / name)
        for name in EXPECTED_FILES - {"human_review.json"}
    }


def test_behavior_review_rejects_unknown_label(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e0_0001")
    _populate_complete_bundle(writer)
    episode_dir = writer.publish()

    with pytest.raises(ValueError, match="label"):
        review_episode_bundle(
            episode_dir,
            status="approved",
            reviewer="project_owner",
            labels=("delete_failed_episode",),
            notes="not allowed",
        )


class _FakeVideoWriter:
    def __init__(self, opened: bool, output_path: str | None = None):
        self._opened = opened
        self.output_path = output_path
        self.frames: list[np.ndarray] = []
        self.released = False

    def isOpened(self) -> bool:
        return self._opened

    def write(self, frame: np.ndarray) -> None:
        self.frames.append(np.asarray(frame).copy())

    def release(self) -> None:
        self.released = True
        if self._opened and self.output_path is not None:
            Path(self.output_path).write_bytes(b"encoded-video")


class _FakeCV2:
    COLOR_RGB2BGR = 1

    def __init__(self, opened: bool):
        self.writer = _FakeVideoWriter(opened)
        self.arguments = None

    @staticmethod
    def VideoWriter_fourcc(*codec: str) -> int:
        assert codec == ("m", "p", "4", "v")
        return 1234

    def VideoWriter(self, *arguments: object) -> _FakeVideoWriter:
        self.arguments = arguments
        self.writer.output_path = str(arguments[0])
        return self.writer

    @staticmethod
    def cvtColor(frame: np.ndarray, conversion: int) -> np.ndarray:
        assert conversion == _FakeCV2.COLOR_RGB2BGR
        return frame[..., ::-1]


def test_video_writer_requires_open_encoder_and_uses_ten_fps(tmp_path: Path):
    frames = [np.zeros((24, 32, 3), dtype=np.uint8)]
    closed_cv2 = _FakeCV2(opened=False)

    with pytest.raises(RuntimeError, match="could not open"):
        write_rgb_video(tmp_path / "closed.mp4", frames, cv2_module=closed_cv2)
    assert closed_cv2.writer.released is True

    opened_cv2 = _FakeCV2(opened=True)
    result = write_rgb_video(tmp_path / "opened.mp4", frames, cv2_module=opened_cv2)

    assert result == tmp_path / "opened.mp4"
    assert opened_cv2.arguments[2] == 10.0
    assert opened_cv2.arguments[3] == (32, 24)
    assert opened_cv2.writer.released is True
    assert len(opened_cv2.writer.frames) == 1


def test_benchmark_entrypoints_import_without_loading_isaac():
    from scripts import review_benchmark_episode, run_pointgoal_benchmark

    assert callable(review_benchmark_episode.main)
    assert callable(run_pointgoal_benchmark.main)


def test_execution_state_artifact_exposes_state_and_action_aligned_diagnostics():
    from scripts.run_pointgoal_benchmark import _execution_state_arrays

    result = SimpleNamespace(
        yaw_world_rad=np.zeros(3, dtype=np.float64),
        linear_velocity_world_m_s=np.zeros((3, 3), dtype=np.float32),
        angular_velocity_world_rad_s=np.zeros((3, 3), dtype=np.float32),
        joint_position_rad=np.zeros((3, 3), dtype=np.float32),
        joint_velocity_rad_s=np.zeros((3, 3), dtype=np.float32),
        applied_action=np.zeros((3, 2), dtype=np.float32),
        policy_reference_index=np.asarray((4, 5), dtype=np.int64),
        policy_cross_track_error_m=np.asarray((0.1, 0.2), dtype=np.float64),
        policy_active_drive_direction=np.asarray((1, -1), dtype=np.int8),
        policy_desired_drive_direction=np.asarray((1, -1), dtype=np.int8),
        policy_pending_drive_direction=np.asarray((0, -1), dtype=np.int8),
        policy_stopped_cycles=np.asarray((0, 1), dtype=np.int64),
        policy_braking_state_code=np.asarray((1, 2), dtype=np.int8),
        dynamic_reference_index=np.asarray((4, 5), dtype=np.int64),
        dynamic_position_error_m=np.asarray((0.1, 0.2), dtype=np.float64),
        dynamic_yaw_error_rad=np.asarray((0.01, 0.02), dtype=np.float64),
        dynamic_auxiliary_error_rad=np.asarray((0.03, 0.04), dtype=np.float64),
        simulation_time_s=np.asarray((0.0, 0.1, 0.2), dtype=np.float64),
        physics_step_index=np.asarray((0, 5, 10), dtype=np.int64),
        raw_simulation_time_s=np.asarray((8.0, 8.1, 8.2), dtype=np.float64),
        raw_physics_step_index=np.asarray((200, 205, 210), dtype=np.int64),
        roll_world_rad=np.asarray((0.01, 0.02, 0.03), dtype=np.float64),
    )

    arrays = _execution_state_arrays(result)

    assert set(arrays) == {
        "yaw_world_rad",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "applied_action",
        "policy_reference_index",
        "policy_cross_track_error_m",
        "policy_active_drive_direction",
        "policy_desired_drive_direction",
        "policy_pending_drive_direction",
        "policy_stopped_cycles",
        "policy_braking_state_code",
        "dynamic_reference_index",
        "dynamic_position_error_m",
        "dynamic_yaw_error_rad",
        "dynamic_auxiliary_error_rad",
        "simulation_time_s",
        "physics_step_index",
        "raw_simulation_time_s",
        "raw_physics_step_index",
        "roll_world_rad",
    }
    assert arrays["yaw_world_rad"].shape == (3,)
    np.testing.assert_array_equal(arrays["simulation_time_s"], (0.0, 0.1, 0.2))
    np.testing.assert_array_equal(arrays["physics_step_index"], (0, 5, 10))
    np.testing.assert_array_equal(
        arrays["raw_simulation_time_s"], (8.0, 8.1, 8.2)
    )
    np.testing.assert_array_equal(
        arrays["raw_physics_step_index"], (200, 205, 210)
    )
    np.testing.assert_array_equal(arrays["roll_world_rad"], (0.01, 0.02, 0.03))
    assert arrays["policy_reference_index"].shape == (2,)
    assert arrays["dynamic_reference_index"].shape == (2,)


def test_trajectory_panel_composition_uses_cv2_without_pillow(tmp_path: Path):
    from scripts.run_pointgoal_benchmark import _combine_vertical_panels

    class FakeCV2:
        IMREAD_COLOR = 1

        def __init__(self):
            self.written = None

        def imread(self, path, mode):
            assert mode == self.IMREAD_COLOR
            height = 2 if str(path).endswith("task.png") else 3
            return np.full((height, 4, 3), height, dtype=np.uint8)

        def imwrite(self, path, image):
            self.written = (Path(path), np.asarray(image).copy())
            return True

    cv2 = FakeCV2()
    output = tmp_path / "combined.png"

    result = _combine_vertical_panels(
        tmp_path / "task.png",
        tmp_path / "actual.png",
        output,
        cv2_module=cv2,
    )

    assert result == output
    assert cv2.written[0] == output
    assert cv2.written[1].shape == (5, 4, 3)
    assert np.all(cv2.written[1][:2] == 2)
    assert np.all(cv2.written[1][2:] == 3)


def test_isaac_runtime_config_is_pure_and_validates_physical_values(tmp_path: Path):
    from benchmark.isaac_runtime import IsaacRuntimeConfig

    config = IsaacRuntimeConfig(
        scene_root=tmp_path / "scenes",
        contract_path=tmp_path / "contract.yaml",
        gui=True,
        record_video=True,
    )

    assert config.scene_root == tmp_path / "scenes"
    assert config.ball_radius_m == pytest.approx(0.4)
    assert config.safety_margin_m == pytest.approx(0.1)
    with pytest.raises(ValueError, match="ball_radius"):
        IsaacRuntimeConfig(
            scene_root=tmp_path,
            contract_path=tmp_path / "contract.yaml",
            gui=False,
            record_video=True,
            ball_radius_m=0.0,
        )
    with pytest.raises(ValueError, match="safety_margin"):
        IsaacRuntimeConfig(
            scene_root=tmp_path,
            contract_path=tmp_path / "contract.yaml",
            gui=False,
            record_video=True,
            safety_margin_m=-0.1,
        )


def test_contact_filter_groups_use_explicit_scene_colliders_for_every_robot_body():
    from benchmark.isaac_runtime import _contact_filter_groups

    robot_paths = ["/World/BallRobot/base", "/World/BallRobot/wheel"]
    scene_colliders = [
        "/World/Scene/Divider/divider_00",
        "/World/Scene/BoundaryLeft/left_00",
    ]

    assert _contact_filter_groups(robot_paths, scene_colliders) == [
        scene_colliders,
        scene_colliders,
    ]


def test_scene_obstacle_collision_paths_exclude_only_ground_subtree():
    from benchmark.isaac_runtime import _scene_obstacle_collision_paths

    scene_colliders = [
        "/World/Scene/defaultGroundPlane",
        "/World/Scene/defaultGroundPlane/Collision",
        "/World/Scene/defaultGroundPlaneExtra",
        "/World/Scene/Divider/divider_00",
    ]

    assert _scene_obstacle_collision_paths("/World/Scene", scene_colliders) == [
        "/World/Scene/defaultGroundPlaneExtra",
        "/World/Scene/Divider/divider_00",
    ]
    assert _scene_obstacle_collision_paths(
        "/World/Scene", ["/World/Scene/Divider/divider_00"]
    ) == ["/World/Scene/Divider/divider_00"]
    assert _scene_obstacle_collision_paths(
        "/World/Scene", ["/World/Scene/defaultGroundPlane"]
    ) == []


def test_physical_contact_without_obstacle_filter_is_false_but_available():
    from benchmark.isaac_runtime import IsaacBenchmarkRuntime

    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._contacts = None
    runtime._contact_available = True

    assert runtime._physical_contact() is False
    assert runtime.contact_measurement_available is True


@pytest.mark.parametrize(
    ("force_vector", "expected"),
    [((0.0, 0.0, 1.01), True), ((0.0, 0.0, 1.0), False)],
)
def test_physical_contact_keeps_threshold_for_filtered_obstacle_forces(
    force_vector: tuple[float, float, float], expected: bool
):
    from benchmark.isaac_runtime import IsaacBenchmarkRuntime

    class ContactSensor:
        def get_contact_force_matrix(self, *, dt: float) -> np.ndarray:
            assert dt == pytest.approx(1.0 / 50.0)
            return np.asarray([[force_vector]], dtype=np.float64)

    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._contacts = ContactSensor()
    runtime._contract = SimpleNamespace(schedule=SimpleNamespace(physics_hz=50))
    runtime._contact_available = True

    assert runtime._physical_contact() is expected
    assert runtime.contact_measurement_available is True


def test_terminal_video_flush_advances_one_camera_period_without_mutating_episode_state():
    from benchmark.isaac_runtime import IsaacBenchmarkRuntime

    class FakeWorld:
        def __init__(self) -> None:
            self.step_calls: list[bool] = []

        def step(self, *, render: bool) -> None:
            self.step_calls.append(render)

    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._episode = object()
    runtime._world = FakeWorld()
    runtime.config = SimpleNamespace(record_video=True)
    runtime._left_camera = "left"
    runtime._right_camera = "right"
    runtime._third_camera = "third"
    runtime._get_rgb = lambda camera: np.full((1, 1, 3), len(camera), dtype=np.uint8)
    runtime._captured_frames = {"third_person": [], "stereo_view": []}

    runtime.flush_terminal_video_frames()

    assert runtime._world.step_calls == [True] * 5
    assert len(runtime._captured_frames["third_person"]) == 5
    assert len(runtime._captured_frames["stereo_view"]) == 5
    assert runtime._captured_frames["third_person"][0][0, 0, 0] == len("third")
    assert runtime._captured_frames["stereo_view"][0].shape == (1, 2, 3)


def test_runner_resolves_and_hash_checks_frozen_occupancy(tmp_path: Path):
    from scripts.run_pointgoal_benchmark import _resolve_occupancy_path

    occupancy_path = tmp_path / "truth" / "scene.npz"
    occupancy_path.parent.mkdir()
    np.savez(
        occupancy_path,
        grid=np.zeros((4, 4), dtype=np.uint8),
        global_origin_xy=np.zeros(2, dtype=np.float64),
        resolution_m=np.asarray(0.1),
    )
    episode = BenchmarkEpisode(
        episode_id="strict_occupancy",
        split="golden",
        difficulty="E0",
        scene_uri="${SCENE_ROOT}/scene.usd",
        scene_sha256="01" * 32,
        occupancy_sha256=artifact_sha256(occupancy_path),
        start_position_world_m=(0.0, 0.0, 0.45),
        start_quaternion_world_wxyz=(1.0, 0.0, 0.0, 0.0),
        goal_position_world_m=(1.0, 0.0, 0.45),
        euclidean_distance_m=1.0,
        oracle_path_length_m=1.0,
        seed=3072,
        human_review=HumanReview("approved", "owner", "fixture"),
        classification_evidence={"line_is_free": True},
    )
    index_path = tmp_path / "scene_index.json"
    index_path.write_text(
        json.dumps(
            {
                "scenes": [
                    {
                        "scene_uri": episode.scene_uri,
                        "occupancy_uri": "${DATA_ROOT}/truth/scene.npz",
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    assert _resolve_occupancy_path(index_path, episode, tmp_path) == occupancy_path

    occupancy_path.write_bytes(b"modified")
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        _resolve_occupancy_path(index_path, episode, tmp_path)
