"""CPU-only checks for Round 4 frozen-reference open-plane replay."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import run_round4_fixed_reference as fixed_reference  # noqa: E402


def test_reference_transform_preserves_geometry_commands_and_direction(tmp_path: Path):
    yaw0 = 0.7
    pose = np.asarray(
        [[4.0, -2.0, yaw0], [4.5, -1.8, yaw0 + 0.2], [4.8, -1.2, yaw0 + 0.4]],
        dtype=np.float64,
    )
    command = np.asarray([[0.5, -0.2], [0.5, -0.2], [-0.5, 0.2]])
    direction = np.asarray([1, 1, -1], dtype=np.int8)
    source = tmp_path / "states.npz"
    np.savez(
        source,
        kinodynamic_reference_pose=pose,
        kinodynamic_reference_command=command,
        kinodynamic_reference_drive_direction=direction,
    )

    reference, provenance = fixed_reference.load_transformed_reference(
        source, case_id="hard_0_e1_0002"
    )

    assert reference.points_xy[0] == pytest.approx([0.0, 0.0])
    assert reference.yaw_rad[0] == pytest.approx(0.0)
    assert np.linalg.norm(np.diff(reference.points_xy, axis=0), axis=1) == pytest.approx(
        np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1)
    )
    assert [row.main_command_rad_s for row in reference.waypoints] == pytest.approx(
        command[:, 0]
    )
    assert [row.auxiliary_angle_rad for row in reference.waypoints] == pytest.approx(
        command[:, 1]
    )
    assert [row.drive_direction for row in reference.waypoints] == [
        "forward",
        "forward",
        "reverse",
    ]
    assert reference.segment_boundaries == (1, 2)
    assert provenance["transform"]["rotation_rad"] == pytest.approx(-yaw0)


def test_reference_loader_rejects_missing_or_misaligned_arrays(tmp_path: Path):
    source = tmp_path / "bad.npz"
    np.savez(
        source,
        kinodynamic_reference_pose=np.zeros((2, 3)),
        kinodynamic_reference_command=np.zeros((1, 2)),
        kinodynamic_reference_drive_direction=np.ones(2, dtype=np.int8),
    )

    with pytest.raises(ValueError, match="align"):
        fixed_reference.load_transformed_reference(source, case_id="hard_0_e1_0002")


def test_frozen_reference_loader_is_independent_of_drive_profile(tmp_path: Path):
    pose = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    command = np.asarray([[0.5, 0.0], [0.5, 0.0]])
    source = tmp_path / "states.npz"
    np.savez(
        source,
        kinodynamic_reference_pose=pose,
        kinodynamic_reference_command=command,
        kinodynamic_reference_drive_direction=np.ones(2, dtype=np.int8),
    )

    reference, _ = fixed_reference.load_transformed_reference(
        source, case_id="hard_0_e1_0004"
    )

    assert reference.points_xy.tolist() == [[0.0, 0.0], [1.0, 0.0]]
    assert math.isclose(reference.path_length_m, 1.0)
