"""Isaac-free tests for the coupled ball-drive dynamics calibration."""

from __future__ import annotations

import math
from pathlib import Path
import json
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from probe_drive_dynamics import (  # noqa: E402
    CalibrationCase,
    _parse_args,
    _render_report_in_clean_process,
    calibration_cases_from_contract,
    render_calibration_report,
    run_calibration,
    summarize_braking_trial,
    summarize_motion_trial,
    summarize_steering_transient,
    trial_stages,
)
import probe_drive_dynamics as drive_probe  # noqa: E402


def _contract(**overrides: object) -> SimpleNamespace:
    values = {
        "action_protocol_id": "ball_action_v2_aux30",
        "action_names": (
            "main_joint_velocity_cmd",
            "auxiliary_joint_angle_cmd",
        ),
        "action_minimum": (-2.0, math.radians(-30.0)),
        "action_maximum": (2.0, math.radians(30.0)),
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_calibration_matrix_covers_main_auxiliary_braking_and_transients():
    cases = calibration_cases_from_contract(_contract())

    assert len(cases) == 54
    covered_conditions = {
        (case.mode, case.main_command_rad_s, case.auxiliary_command_rad)
        for case in cases
    }
    main_commands = {-1.5, -1.0, -0.5, 0.5, 1.0, 1.5}
    auxiliary_commands = {
        math.radians(value) for value in (-30.0, -20.0, -10.0, 10.0, 20.0, 30.0)
    }
    assert {
        (main, auxiliary)
        for mode, main, auxiliary in covered_conditions
        if mode == "steady_turn"
    } == {
        (main, auxiliary)
        for main in main_commands
        for auxiliary in auxiliary_commands
    }
    assert {
        main
        for mode, main, auxiliary in covered_conditions
        if mode == "braking" and auxiliary == 0.0
    } == main_commands
    assert {
        (main, auxiliary)
        for mode, main, auxiliary in covered_conditions
        if mode == "steering_step"
    } == {
        (main, math.radians(auxiliary))
        for main in (0.5, 1.0, 1.5)
        for auxiliary in (-20.0, 20.0)
    }
    assert len({case.case_id for case in cases}) == len(cases)


def test_calibration_matrix_rejects_incompatible_action_contract():
    with pytest.raises(ValueError, match="ball_action_v2_aux30"):
        calibration_cases_from_contract(
            _contract(action_protocol_id="ball_action_v1_aux16")
        )

    with pytest.raises(ValueError, match="bounds"):
        calibration_cases_from_contract(
            _contract(action_maximum=(1.0, math.radians(30.0)))
        )


def test_motion_summary_recovers_speed_yaw_rate_curvature_and_response_times():
    summary = summarize_motion_trial(
        case_id="turn_v1p0_aux10",
        mode="steady_turn",
        time_s=[0.0, 1.0, 2.0, 3.0, 4.0],
        phase=["drive"] * 5,
        main_target_rad_s=[1.0] * 5,
        auxiliary_target_rad=[0.2] * 5,
        main_joint_velocity_rad_s=[0.0, 0.6, 0.95, 1.0, 1.0],
        auxiliary_joint_rad=[0.0, 0.12, 0.19, 0.2, 0.2],
        yaw_rad=[0.0, 0.1, 0.2, 0.3, 0.4],
        position_xy_m=[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0], [4.0, 0.0]],
        linear_velocity_world_xy_m_s=[[1.0, 0.0]] * 5,
    )

    assert summary["steady_main_joint_velocity_rad_s"] == pytest.approx(1.0)
    assert summary["steady_auxiliary_joint_rad"] == pytest.approx(0.2)
    assert summary["steady_forward_speed_m_s"] == pytest.approx(1.0, abs=0.03)
    assert summary["steady_yaw_rate_rad_s"] == pytest.approx(0.1)
    assert summary["steady_curvature_1_m"] == pytest.approx(0.1, abs=0.003)
    assert summary["signed_turning_radius_m"] == pytest.approx(10.0, abs=0.3)
    assert summary["main_time_to_90_percent_s"] == pytest.approx(2.0)
    assert summary["auxiliary_time_to_90_percent_s"] == pytest.approx(2.0)
    assert len(summary["samples"]) == 5


def test_straight_motion_summary_reports_infinite_turning_radius():
    summary = summarize_motion_trial(
        case_id="straight_v1p0",
        mode="straight",
        time_s=[0.0, 1.0, 2.0, 3.0],
        phase=["drive"] * 4,
        main_target_rad_s=[1.0] * 4,
        auxiliary_target_rad=[0.0] * 4,
        main_joint_velocity_rad_s=[0.0, 0.9, 1.0, 1.0],
        auxiliary_joint_rad=[0.0] * 4,
        yaw_rad=[0.0] * 4,
        position_xy_m=[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]],
        linear_velocity_world_xy_m_s=[[1.0, 0.0]] * 4,
    )

    assert summary["steady_curvature_1_m"] == pytest.approx(0.0)
    assert summary["signed_turning_radius_m"] is None


def test_braking_summary_measures_stop_time_and_distance_after_zero_command():
    summary = summarize_braking_trial(
        time_s=[0.0, 1.0, 2.0, 3.0, 4.0],
        phase=["drive", "drive", "brake", "brake", "brake"],
        position_xy_m=[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.6, 0.0], [2.65, 0.0]],
        planar_speed_m_s=[1.0, 1.0, 0.8, 0.3, 0.02],
        stop_speed_threshold_m_s=0.05,
    )

    assert summary["brake_start_time_s"] == pytest.approx(2.0)
    assert summary["initial_braking_speed_m_s"] == pytest.approx(0.8)
    assert summary["stop_time_s"] == pytest.approx(2.0)
    assert summary["stopping_distance_m"] == pytest.approx(0.65)
    assert summary["stopped_within_trial"] is True


def test_steering_transient_measures_rise_and_release_delays():
    summary = summarize_steering_transient(
        time_s=[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        phase=["baseline", "steer", "steer", "steer", "release", "release", "release"],
        auxiliary_command_rad=0.2,
        auxiliary_joint_rad=[0.0, 0.05, 0.15, 0.2, 0.15, 0.05, 0.0],
        yaw_rate_rad_s=[0.0, 0.01, 0.04, 0.06, 0.05, 0.02, 0.0],
    )

    assert summary["auxiliary_rise_time_90_percent_s"] == pytest.approx(2.0)
    assert summary["auxiliary_release_time_10_percent_s"] == pytest.approx(2.0)
    assert summary["peak_abs_yaw_rate_rad_s"] == pytest.approx(0.06)
    assert summary["returned_to_neutral"] is True


def test_trial_stages_apply_and_remove_commands_for_each_mode():
    braking = trial_stages(
        CalibrationCase("brake", "braking", 1.0, 0.0),
        main_acceleration_rad_s2=0.5,
        auxiliary_rate_rad_s=math.radians(30.0),
    )
    steering = trial_stages(
        CalibrationCase("step", "steering_step", 1.0, math.radians(20.0)),
        main_acceleration_rad_s2=0.5,
        auxiliary_rate_rad_s=math.radians(30.0),
    )

    assert [stage.phase for stage in braking] == ["drive", "brake"]
    assert braking[0].main_target_rad_s == pytest.approx(1.0)
    assert braking[-1].main_target_rad_s == pytest.approx(0.0)
    assert [stage.phase for stage in steering] == ["baseline", "steer", "release"]
    assert steering[1].auxiliary_target_rad == pytest.approx(math.radians(20.0))
    assert steering[-1].auxiliary_target_rad == pytest.approx(0.0)
    assert all(stage.duration_s > 0.0 for stage in (*braking, *steering))


def test_drive_dynamics_module_import_does_not_start_isaac():
    program = f"""
import sys
sys.path.insert(0, {str(SCRIPTS_DIR)!r})
import probe_drive_dynamics
assert 'isaacsim' not in sys.modules
assert not any(name == 'omni' or name.startswith('omni.') for name in sys.modules)
"""

    completed = subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr


def _report_fixture() -> dict[str, object]:
    return {
        "probe": "coupled_drive_dynamics",
        "cases": [
            {
                "case_id": "straight_main_pos1p0",
                "mode": "straight",
                "main_command_rad_s": 1.0,
                "auxiliary_command_rad": 0.0,
                "steady_main_joint_velocity_rad_s": 0.98,
                "steady_forward_speed_m_s": 0.42,
                "steady_curvature_1_m": 0.0,
                "signed_turning_radius_m": None,
            },
            {
                "case_id": "turn_main_pos1p0_aux_pos20p0deg",
                "mode": "steady_turn",
                "main_command_rad_s": 1.0,
                "auxiliary_command_rad": math.radians(20.0),
                "steady_main_joint_velocity_rad_s": 0.97,
                "steady_forward_speed_m_s": 0.40,
                "steady_curvature_1_m": -0.5,
                "signed_turning_radius_m": -2.0,
            },
            {
                "case_id": "brake_main_pos1p0",
                "mode": "braking",
                "main_command_rad_s": 0.0,
                "auxiliary_command_rad": 0.0,
                "steady_main_joint_velocity_rad_s": 0.0,
                "steady_forward_speed_m_s": 0.0,
                "steady_curvature_1_m": None,
                "signed_turning_radius_m": None,
                "braking": {
                    "initial_braking_speed_m_s": 0.42,
                    "stopping_distance_m": 0.31,
                    "stop_time_s": 0.8,
                    "stopped_within_trial": True,
                },
            },
        ],
    }


def _complete_signed_report_fixture() -> dict[str, object]:
    cases: list[dict[str, object]] = []
    for main_command in (-1.5, -1.0, -0.5, 0.5, 1.0, 1.5):
        direction_sign = 1.0 if main_command > 0.0 else -1.0
        speed = abs(main_command) * 0.4
        cases.append(
            {
                "case_id": f"straight_{main_command:+.1f}",
                "mode": "straight",
                "main_command_rad_s": main_command,
                "auxiliary_command_rad": 0.0,
                "steady_main_joint_velocity_rad_s": 0.98 * main_command,
                "steady_forward_speed_m_s": direction_sign * speed,
                "steady_curvature_1_m": 0.0,
                "signed_turning_radius_m": None,
            }
        )
        for auxiliary_deg in (-30.0, -20.0, -10.0, 10.0, 20.0, 30.0):
            curvature = -direction_sign * auxiliary_deg / 50.0
            cases.append(
                {
                    "case_id": f"turn_{main_command:+.1f}_{auxiliary_deg:+.0f}",
                    "mode": "steady_turn",
                    "main_command_rad_s": main_command,
                    "auxiliary_command_rad": math.radians(auxiliary_deg),
                    "steady_main_joint_velocity_rad_s": 0.98 * main_command,
                    "steady_forward_speed_m_s": direction_sign * speed,
                    "steady_curvature_1_m": curvature,
                    "signed_turning_radius_m": 1.0 / curvature,
                }
            )
        cases.append(
            {
                "case_id": f"brake_{main_command:+.1f}",
                "mode": "braking",
                "main_command_rad_s": main_command,
                "auxiliary_command_rad": 0.0,
                "steady_main_joint_velocity_rad_s": 0.0,
                "steady_forward_speed_m_s": 0.0,
                "steady_curvature_1_m": None,
                "signed_turning_radius_m": None,
                "braking": {
                    "initial_braking_speed_m_s": speed,
                    "stopping_distance_m": speed * speed + 0.1,
                    "stop_time_s": speed + 0.2,
                    "stopped_within_trial": True,
                },
            }
        )
    return {
        "schema_version": "1.0",
        "probe": "coupled_drive_dynamics",
        "measurement_timestamp_utc": "2026-09-12T07:30:00+00:00",
        "action_protocol_id": "ball_action_v2_aux30",
        "physics_hz": 50,
        "settle_steps": 50,
        "case_count": len(cases),
        "controller": {
            "main_acceleration_limit_rad_s2": 0.5,
            "main_derivative_gain": 80.0,
            "auxiliary_kp": 300.0,
            "auxiliary_kd": 60.0,
            "auxiliary_target_rate_limit_rad_s": math.radians(30.0),
        },
        "cases": cases,
    }


def test_compact_profile_preserves_measured_signed_steering_and_braking():
    assert hasattr(drive_probe, "compact_drive_profile"), (
        "calibration report needs a validated compact profile exporter"
    )

    profile = drive_probe.compact_drive_profile(_complete_signed_report_fixture())

    assert profile["schema_version"] == "drive_dynamics_v1"
    assert profile["action_protocol_id"] == "ball_action_v2_aux30"
    assert profile["measurement_timestamp_utc"] == "2026-09-12T07:30:00+00:00"
    assert len(profile["steering_samples"]) == 42
    assert len(profile["braking_samples"]) == 6
    assert {row["drive_direction"] for row in profile["steering_samples"]} == {
        "forward",
        "reverse",
    }
    assert {row["drive_direction"] for row in profile["braking_samples"]} == {
        "forward",
        "reverse",
    }
    assert all(row["speed_m_s"] >= 0.0 for row in profile["braking_samples"])


def test_compact_profile_rejects_incomplete_reverse_measurements():
    assert hasattr(drive_probe, "compact_drive_profile"), (
        "calibration report needs a validated compact profile exporter"
    )
    report = _complete_signed_report_fixture()
    report["cases"] = [
        case
        for case in report["cases"]
        if not (
            case["mode"] == "steady_turn"
            and case["main_command_rad_s"] == -1.0
            and case["auxiliary_command_rad"] == math.radians(20.0)
        )
    ]
    report["case_count"] = len(report["cases"])

    with pytest.raises(ValueError, match="complete signed steering matrix"):
        drive_probe.compact_drive_profile(report)


def test_compact_profile_rejects_non_finite_and_duplicate_operating_points():
    assert hasattr(drive_probe, "compact_drive_profile"), (
        "calibration report needs a validated compact profile exporter"
    )
    non_finite = _complete_signed_report_fixture()
    non_finite["cases"][0]["steady_forward_speed_m_s"] = math.nan
    with pytest.raises(ValueError, match="finite"):
        drive_probe.compact_drive_profile(non_finite)

    duplicate = _complete_signed_report_fixture()
    duplicate["cases"].append(dict(duplicate["cases"][0]))
    duplicate["case_count"] = len(duplicate["cases"])
    with pytest.raises(ValueError, match="duplicate"):
        drive_probe.compact_drive_profile(duplicate)


def test_compact_profile_rejects_incompatible_controller_metadata():
    assert hasattr(drive_probe, "compact_drive_profile"), (
        "calibration report needs a validated compact profile exporter"
    )
    report = _complete_signed_report_fixture()
    report["controller"]["main_acceleration_limit_rad_s2"] = 0.8

    with pytest.raises(ValueError, match="controller metadata"):
        drive_probe.compact_drive_profile(report)


def test_compact_profile_writer_creates_reloadable_json(tmp_path: Path):
    output = tmp_path / "drive_dynamics_v1.json"

    returned = drive_probe.write_compact_drive_profile(
        output, _complete_signed_report_fixture()
    )

    assert returned == output
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "drive_dynamics_v1"
    assert len(payload["steering_samples"]) == 42


def test_render_calibration_report_writes_summary_png(tmp_path: Path):
    output = tmp_path / "drive_dynamics_calibration.png"

    returned = render_calibration_report(output, _report_fixture())

    assert returned == output
    with Image.open(output) as image:
        assert image.format == "PNG"
        assert image.width >= 900
        assert image.height >= 600


def test_clean_renderer_subprocess_writes_summary_png(tmp_path: Path):
    report_path = tmp_path / "drive_dynamics_calibration.json"
    report_path.write_text(json.dumps(_report_fixture()), encoding="utf-8")

    output = _render_report_in_clean_process(report_path)

    assert output.is_file()


def test_cli_defaults_to_headless_full_calibration(tmp_path: Path):
    args = _parse_args(["--out", str(tmp_path)])

    assert args.out == tmp_path
    assert args.gui is False
    assert args.main_accel_rad_s2 == pytest.approx(0.5)
    assert args.aux_rate_deg_s == pytest.approx(30.0)


def test_cli_accepts_explicit_compact_profile_destination(tmp_path: Path):
    profile = tmp_path / "drive_dynamics_v1.json"

    args = _parse_args(
        ["--out", str(tmp_path / "raw"), "--profile-out", str(profile)]
    )

    assert args.profile_out == profile


def test_runtime_rejects_invalid_controller_parameters_before_importing_isaac(
    tmp_path: Path,
):
    with pytest.raises(ValueError, match="positive"):
        run_calibration(
            Path("contract.yaml"),
            tmp_path,
            gui=False,
            main_acceleration_rad_s2=0.0,
            auxiliary_kp=300.0,
            auxiliary_kd=60.0,
            auxiliary_rate_rad_s=math.radians(30.0),
        )
