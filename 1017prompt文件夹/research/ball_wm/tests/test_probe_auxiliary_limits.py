"""Isaac-free acceptance tests for the auxiliary steering limit probe."""

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

from probe_auxiliary_limits import (
    _parse_args,
    _render_report_in_clean_process,
    contact_filter_groups,
    evaluate_probe,
    probe_controls_from_contract,
    render_probe_plot,
    slew_limited_target,
    summarize_direction,
)


def _accepted_inputs() -> dict[str, np.ndarray]:
    return {
        "commanded_rad": np.asarray([0.5235988, -0.5235988]),
        "measured_rad": np.asarray([0.520, -0.521]),
        "max_abs_measured_rad": np.asarray([0.522, 0.523]),
        "yaw_delta_rad": np.asarray([0.18, -0.17]),
        "max_abs_roll_rad": np.asarray([0.08, 0.09]),
        "max_aux_speed_rad_s": np.deg2rad([34.0, 35.0]),
        "max_height_deviation_m": np.asarray([0.01, 0.015]),
        "collision": np.asarray([False, False]),
    }


def test_evaluate_probe_accepts_symmetric_aux30_tracking():
    result = evaluate_probe(**_accepted_inputs())

    assert result["passed"] is True
    assert result["joint_limit_ok"] is True
    assert result["tracking_error_ok"] is True
    assert result["opposite_yaw_sign_ok"] is True
    assert result["yaw_magnitude_ok"] is True
    assert result["aux_speed_ok"] is True
    assert result["height_stability_ok"] is True
    assert result["roll_diagnostic_only"] is True
    assert result["collision_free"] is True


@pytest.mark.parametrize(
    ("field", "replacement", "failed_check"),
    [
        ("max_abs_measured_rad", np.deg2rad([30.6, 29.9]), "joint_limit_ok"),
        ("measured_rad", np.deg2rad([25.0, -25.0]), "tracking_error_ok"),
        ("yaw_delta_rad", np.asarray([0.18, 0.17]), "opposite_yaw_sign_ok"),
        ("yaw_delta_rad", np.deg2rad([2.9, -4.0]), "yaw_magnitude_ok"),
        ("max_aux_speed_rad_s", np.deg2rad([40.1, 35.0]), "aux_speed_ok"),
        ("max_height_deviation_m", np.asarray([0.051, 0.02]), "height_stability_ok"),
        ("collision", np.asarray([False, True]), "collision_free"),
    ],
)
def test_evaluate_probe_rejects_each_physical_acceptance_failure(
    field: str,
    replacement: np.ndarray,
    failed_check: str,
):
    inputs = _accepted_inputs()
    inputs[field] = replacement

    result = evaluate_probe(**inputs)

    assert result["passed"] is False
    assert result[failed_check] is False


@pytest.mark.parametrize(
    "field",
    (
        "commanded_rad",
        "measured_rad",
        "max_abs_measured_rad",
        "yaw_delta_rad",
        "max_abs_roll_rad",
        "max_aux_speed_rad_s",
        "max_height_deviation_m",
    ),
)
def test_evaluate_probe_rejects_non_finite_numeric_inputs(field: str):
    inputs = _accepted_inputs()
    inputs[field] = inputs[field].copy()
    inputs[field][0] = math.nan

    with pytest.raises(ValueError, match="finite"):
        evaluate_probe(**inputs)


def test_root_roll_is_diagnostic_for_a_rolling_spherical_base():
    inputs = _accepted_inputs()
    inputs["max_abs_roll_rad"] = np.deg2rad([120.0, 95.0])

    result = evaluate_probe(**inputs)

    assert result["passed"] is True
    assert result["roll_diagnostic_only"] is True
    assert result["max_abs_roll_rad"] == pytest.approx(np.deg2rad([120.0, 95.0]))


def test_evaluate_probe_rejects_wrong_direction_count():
    inputs = _accepted_inputs()
    inputs["measured_rad"] = np.asarray([0.52])

    with pytest.raises(ValueError, match=r"shape \(2,\)"):
        evaluate_probe(**inputs)


def test_probe_module_import_does_not_load_isaac():
    program = f"""
import sys
sys.path.insert(0, {str(SCRIPTS_DIR)!r})
import probe_auxiliary_limits
assert 'isaacsim' not in sys.modules
assert not any(name == 'omni' or name.startswith('omni.') for name in sys.modules)
assert not any(name == 'PIL' or name.startswith('PIL.') for name in sys.modules)
"""

    completed = subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr


def test_summarize_direction_unwraps_yaw_and_retains_step_traces():
    summary = summarize_direction(
        label="positive",
        main_command_rad_s=0.5,
        main_target_rad_s=[0.0, 0.02, 0.04],
        auxiliary_command_rad=math.radians(30.0),
        joint_target_rad=[0.0, 0.25, 0.52],
        joint_rad=[0.0, 0.52, 0.52],
        yaw_rad=np.deg2rad([179.0, -179.0, -170.0]),
        roll_rad=[0.0, -0.08, 0.1],
        joint_velocity_rad_s=[0.0, 0.5, 0.6],
        base_z_m=[0.45, 0.46, 0.44],
        vertical_velocity_m_s=[0.0, 0.1, -0.2],
        contact_force_norm=[0.0, 11.0, 12.0],
        collision=[False, False, False],
        position_xy=[[0.0, 0.0], [0.1, 0.02], [0.2, 0.05]],
    )

    assert summary["measured_auxiliary_rad"] == pytest.approx(0.52)
    assert summary["max_abs_auxiliary_rad"] == pytest.approx(0.52)
    assert summary["yaw_delta_rad"] == pytest.approx(math.radians(11.0))
    assert summary["max_abs_roll_rad"] == pytest.approx(0.1)
    assert summary["max_aux_speed_rad_s"] == pytest.approx(0.6)
    assert summary["max_height_deviation_m"] == pytest.approx(0.01)
    assert summary["max_abs_vertical_velocity_m_s"] == pytest.approx(0.2)
    assert summary["max_contact_force_norm"] == pytest.approx(12.0)
    assert summary["collision"] is False
    assert len(summary["samples"]) == 3
    assert summary["samples"][-1]["main_target_rad_s"] == pytest.approx(0.04)
    assert summary["samples"][-1]["joint_target_rad"] == pytest.approx(0.52)
    assert summary["samples"][-1]["position_xy_m"] == pytest.approx([0.2, 0.05])


def test_slew_limited_target_moves_at_configured_rate_without_overshoot():
    dt = 0.02
    rate = math.radians(30.0)

    first = slew_limited_target(0.0, math.radians(30.0), rate, dt)
    negative = slew_limited_target(0.0, math.radians(-30.0), rate, dt)
    final = slew_limited_target(math.radians(29.8), math.radians(30.0), rate, dt)

    assert first == pytest.approx(math.radians(0.6))
    assert negative == pytest.approx(math.radians(-0.6))
    assert final == pytest.approx(math.radians(30.0))


def test_slew_limiter_ramps_main_velocity_at_configured_acceleration():
    dt = 0.02

    first = slew_limited_target(0.0, 0.5, 0.5, dt)
    next_step = slew_limited_target(first, 0.5, 0.5, dt)

    assert first == pytest.approx(0.01)
    assert next_step == pytest.approx(0.02)


def test_cli_defaults_to_selected_stable_pid_and_rate_limit(tmp_path: Path):
    args = _parse_args(["--out", str(tmp_path)])

    assert args.aux_kp == pytest.approx(300.0)
    assert args.aux_kd == pytest.approx(60.0)
    assert args.aux_rate_deg_s == pytest.approx(30.0)
    assert args.main_accel_rad_s2 == pytest.approx(0.5)


def test_render_probe_plot_writes_both_direction_trajectories(tmp_path: Path):
    records = [
        {
            "label": "positive",
            "samples": [
                {"position_xy_m": [0.0, 0.0]},
                {"position_xy_m": [0.5, 0.2]},
            ],
        },
        {
            "label": "negative",
            "samples": [
                {"position_xy_m": [0.0, 0.0]},
                {"position_xy_m": [0.5, -0.2]},
            ],
        },
    ]
    output = tmp_path / "steering_probe.png"

    returned = render_probe_plot(output, records, passed=True)

    assert returned == output
    with Image.open(output) as image:
        assert image.format == "PNG"
        assert image.width >= 600
        assert image.height >= 400


def test_clean_renderer_subprocess_writes_png_from_saved_report(tmp_path: Path):
    report_path = tmp_path / "steering_probe.json"
    report_path.write_text(
        json.dumps(
            {
                "passed": True,
                "directions": [
                    {
                        "label": "positive",
                        "samples": [
                            {"position_xy_m": [0.0, 0.0]},
                            {"position_xy_m": [0.2, 0.1]},
                        ],
                    },
                    {
                        "label": "negative",
                        "samples": [
                            {"position_xy_m": [0.0, 0.0]},
                            {"position_xy_m": [0.2, -0.1]},
                        ],
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    output = _render_report_in_clean_process(report_path)

    assert output.is_file()


def _aux30_contract(**overrides: object) -> SimpleNamespace:
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


def test_probe_controls_are_derived_from_the_v2_contract():
    controls = probe_controls_from_contract(_aux30_contract())

    assert [label for label, _ in controls] == ["positive", "negative"]
    assert [command for _, command in controls] == pytest.approx(
        [math.radians(30.0), math.radians(-30.0)]
    )


def test_probe_rejects_legacy_action_protocol_before_starting_isaac():
    with pytest.raises(ValueError, match="ball_action_v2_aux30"):
        probe_controls_from_contract(
            _aux30_contract(action_protocol_id="ball_action_v1_aux16")
        )


def test_probe_rejects_v2_contract_with_wrong_physical_bounds():
    with pytest.raises(ValueError, match="30 degrees"):
        probe_controls_from_contract(
            _aux30_contract(
                action_minimum=(-2.0, math.radians(-25.0)),
                action_maximum=(2.0, math.radians(25.0)),
            )
        )


def test_contact_filter_groups_match_each_rigid_sensor_pattern():
    groups = contact_filter_groups(["/World/BallRobot/base", "/World/BallRobot/wheel"])

    assert groups == [[], []]
