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


def test_trial_stages_extend_steady_turn_for_auxiliary_velocity_limit():
    case = CalibrationCase("turn", "steady_turn", 1.5, math.radians(30.0))

    nominal = trial_stages(
        case,
        main_acceleration_rad_s2=0.5,
        auxiliary_rate_rad_s=math.radians(30.0),
    )
    limited = trial_stages(
        case,
        main_acceleration_rad_s2=0.5,
        auxiliary_rate_rad_s=math.radians(30.0),
        auxiliary_velocity_limit_rad_s=0.30,
    )

    assert limited[0].duration_s > nominal[0].duration_s


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


def test_compact_profile_marks_velocity_limited_calibration_as_v2():
    report = _complete_signed_report_fixture()
    report["controller"]["auxiliary_velocity_limit_rad_s"] = 0.30

    profile = drive_probe.compact_drive_profile(report)

    assert profile["schema_version"] == "drive_dynamics_v2"
    assert profile["controller"]["auxiliary_velocity_limit_rad_s"] == pytest.approx(0.30)


def test_compact_profile_monotonicizes_small_curvature_measurement_noise():
    report = _complete_signed_report_fixture()
    rows = [
        case
        for case in report["cases"]
        if case["mode"] == "steady_turn"
        and case["main_command_rad_s"] == 0.5
    ]
    rows[1]["steady_curvature_1_m"] = rows[0]["steady_curvature_1_m"] + 0.02

    profile = drive_probe.compact_drive_profile(report)
    curvatures = [
        row["curvature_1_m"]
        for row in sorted(
            profile["steering_samples"],
            key=lambda row: (row["main_command_rad_s"], row["auxiliary_angle_rad"]),
        )
        if row["main_command_rad_s"] == 0.5
    ]

    assert all(right < left for left, right in zip(curvatures, curvatures[1:]))


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


def test_compact_profile_writer_preserves_velocity_limit_metadata(tmp_path: Path):
    output = tmp_path / "drive_dynamics_v2.json"
    report = _complete_signed_report_fixture()
    report["controller"]["auxiliary_velocity_limit_rad_s"] = 0.30

    drive_probe.write_compact_drive_profile(output, report)

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "drive_dynamics_v2"
    assert payload["controller"]["auxiliary_velocity_limit_rad_s"] == pytest.approx(0.30)


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
    assert args.auxiliary_velocity_limit_rad_s is None


def test_cli_accepts_auxiliary_velocity_limit_for_calibration(tmp_path: Path):
    args = _parse_args(
        [
            "--out",
            str(tmp_path),
            "--auxiliary-velocity-limit-rad-s",
            "0.30",
        ]
    )

    assert args.auxiliary_velocity_limit_rad_s == pytest.approx(0.30)


def test_cli_accepts_round4_manifest(tmp_path: Path):
    manifest = tmp_path / "round4.json"

    args = _parse_args(
        ["--out", str(tmp_path / "run"), "--manifest", str(manifest)]
    )

    assert args.manifest == manifest


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


def _round4_manifest_fixture() -> dict[str, object]:
    return {
        "schema_version": "round4_drive_calibration_v1",
        "action_protocol_id": "ball_action_v2_aux30",
        "drive_profile": {"path": "drive.json", "sha256": "a" * 64},
        "controller": {
            "physics_hz": 50,
            "control_hz": 50,
            "main_acceleration_limit_rad_s2": 0.5,
            "main_derivative_gain": 80.0,
            "auxiliary_kp": 300.0,
            "auxiliary_kd": 60.0,
            "auxiliary_target_rate_limit_rad_s": math.radians(30.0),
            "auxiliary_velocity_limit_rad_s": 0.30,
        },
        "procedure": {
            "steady_window_s": 1.0,
            "maximum_steer_hold_s": 6.0,
            "maximum_release_s": 4.0,
            "diagnostic_radius_m": 12.0,
        },
        "stability": {
            "maximum_auxiliary_median_error_rad": 0.035,
            "maximum_auxiliary_p95_abs_velocity_rad_s": 0.04,
            "minimum_planar_speed_m_s": 0.05,
            "maximum_speed_coefficient_of_variation": 0.10,
            "maximum_abs_yaw_rate_trend_rad_s2": 0.04,
            "minimum_geometric_path_length_m": 0.15,
        },
        "candidate_gates": {
            "minimum_valid_heldout_cases": 1,
            "minimum_curvature_mae_improvement_fraction": 0.2,
            "maximum_speed_mae_regression_fraction": 0.05,
        },
        "cases": [
            {
                "case_id": "core",
                "dataset_role": "fit_core",
                "mode": "steering_step",
                "main_command_rad_s": 0.35,
                "auxiliary_command_rad": math.radians(30.0),
            },
            {
                "case_id": "heldout",
                "dataset_role": "heldout_validation",
                "mode": "steering_step",
                "main_command_rad_s": 0.50,
                "auxiliary_command_rad": math.radians(22.5),
            },
        ],
    }


def test_round4_manifest_preserves_roles_units_and_case_order(tmp_path: Path):
    manifest_path = tmp_path / "round4.json"
    manifest_path.write_text(json.dumps(_round4_manifest_fixture()), encoding="utf-8")

    manifest, cases = drive_probe.load_round4_manifest(manifest_path, _contract())

    assert manifest["controller"]["auxiliary_velocity_limit_rad_s"] == pytest.approx(0.30)
    assert [case.case_id for case in cases] == ["core", "heldout"]
    assert cases[0].dataset_role == "fit_core"
    assert cases[1].dataset_role == "heldout_validation"
    assert cases[0].auxiliary_command_rad == pytest.approx(math.radians(30.0))


def test_round4_manifest_rejects_more_than_declared_run_limits(tmp_path: Path):
    payload = _round4_manifest_fixture()
    payload["cases"] = [
        {
            "case_id": f"core_{index}",
            "dataset_role": "fit_core",
            "mode": "straight",
            "main_command_rad_s": 0.35,
            "auxiliary_command_rad": 0.0,
        }
        for index in range(21)
    ]
    manifest_path = tmp_path / "too_many.json"
    manifest_path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="at most 20 core"):
        drive_probe.load_round4_manifest(manifest_path, _contract())


def test_geometric_curvature_uses_xy_path_and_rejects_low_travel():
    theta = np.linspace(0.0, math.pi / 4.0, 25)
    left_arc = np.column_stack((2.0 * np.sin(theta), 2.0 * (1.0 - np.cos(theta))))

    estimate = drive_probe.estimate_geometric_curvature(
        left_arc, minimum_path_length_m=0.15
    )
    invalid = drive_probe.estimate_geometric_curvature(
        [[0.0, 0.0], [0.01, 0.0], [0.02, 0.0]],
        minimum_path_length_m=0.15,
    )

    assert estimate["valid"] is True
    assert estimate["curvature_1_m"] == pytest.approx(0.5, rel=0.03)
    assert estimate["source"] == "xy_motion_tangent_vs_arc_length"
    assert invalid["valid"] is False
    assert invalid["curvature_1_m"] is None
    assert invalid["invalid_reason"] == "insufficient_path_length"


def test_round4_analysis_marks_nonsteady_and_keeps_null_curvature():
    case = CalibrationCase(
        "unstable", "steering_step", 0.35, math.radians(30.0), "fit_core"
    )
    time = np.linspace(0.0, 2.0, 101)
    phase = np.asarray(["baseline"] * 40 + ["steer"] * 61)
    positions = np.column_stack((0.01 * time, np.zeros_like(time)))
    traces = {
        "time_s": time,
        "phase": phase,
        "requested_main_rad_s": np.full(time.size, 0.35),
        "final_main_rad_s": np.full(time.size, 0.35),
        "main_joint_velocity_rad_s": np.full(time.size, 0.34),
        "requested_auxiliary_rad": np.where(phase == "steer", math.radians(30.0), 0.0),
        "final_auxiliary_rad": np.where(phase == "steer", math.radians(30.0), 0.0),
        "auxiliary_joint_rad": np.zeros(time.size),
        "auxiliary_joint_velocity_rad_s": np.zeros(time.size),
        "position_xy_m": positions,
        "linear_velocity_world_xy_m_s": np.column_stack((np.full(time.size, 0.01), np.zeros(time.size))),
        "body_yaw_rate_rad_s": np.zeros(time.size),
    }

    result = drive_probe.analyze_round4_response(
        case=case,
        traces=traces,
        stability=_round4_manifest_fixture()["stability"],
        steady_window_s=1.0,
    )

    assert result["steady_state_valid"] is False
    assert "auxiliary_target_not_reached" in result["invalid_reasons"]
    assert "low_planar_speed" in result["invalid_reasons"]
    assert result["steady_curvature_1_m"] is None


def test_candidate_evaluation_uses_heldout_rows_only():
    rows = [
        {
            "dataset_role": "fit_core",
            "v1_speed_error_m_s": 9.0,
            "candidate_speed_error_m_s": 0.0,
            "v1_curvature_error_1_m": 9.0,
            "candidate_curvature_error_1_m": 0.0,
            "steady_state_valid": True,
        },
        {
            "dataset_role": "heldout_validation",
            "v1_speed_error_m_s": 0.10,
            "candidate_speed_error_m_s": 0.11,
            "v1_curvature_error_1_m": 0.20,
            "candidate_curvature_error_1_m": 0.19,
            "steady_state_valid": True,
        },
    ]

    result = drive_probe.evaluate_candidate_on_heldout(
        rows, _round4_manifest_fixture()["candidate_gates"]
    )

    assert result["heldout_case_count"] == 1
    assert result["curvature_mae_improvement_fraction"] == pytest.approx(0.05)
    assert result["passed"] is False
