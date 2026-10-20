"""CPU-only contract tests for the PID isolation diagnostic entry point."""

from __future__ import annotations

import importlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import pytest


def _module():
    """Load the optional diagnostic module while keeping the initial RED useful."""

    try:
        return importlib.import_module("scripts.run_pid_isolation")
    except ModuleNotFoundError:
        return None


def test_fixed_command_schedule_has_hand_checked_boundaries():
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    command_for_block = module.command_for_block
    assert command_for_block("forward_zero", 19) == (0.0, 0.0)
    assert command_for_block("forward_zero", 20) == (0.5, 0.0)
    assert command_for_block("forward_zero", 220) == (0.0, 0.0)
    assert command_for_block("forward_pos10_return", 80) == pytest.approx(
        (0.5, math.pi / 18.0)
    )
    assert command_for_block("forward_neg10_return", 80) == pytest.approx(
        (0.5, -math.pi / 18.0)
    )
    assert command_for_block("forward_neg10_return", 160) == (0.5, 0.0)
    assert command_for_block("stationary_pos10_return", 80) == pytest.approx(
        (0.0, math.pi / 18.0)
    )


def test_case_duration_covers_all_seven_spec_cases():
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    expected = {
        "hold_zero": 20.0,
        "forward_zero": 30.0,
        "forward_pos10_return": 30.0,
        "forward_neg10_return": 30.0,
        "stationary_pos10_return": 30.0,
        "closed_loop_lookahead05": 20.0,
        "closed_loop_lookahead10": 20.0,
    }
    assert {case_id: module.case_duration_s(case_id) for case_id in expected} == expected


@pytest.mark.parametrize(
    ("case_id", "block_index"),
    (
        ("unknown", 0),
        ("hold_zero", -1),
        ("hold_zero", 200),
        ("forward_zero", 300),
        ("closed_loop_lookahead05", 0),
    ),
)
def test_schedule_rejects_unknown_negative_over_budget_and_feedback_cases(
    case_id: str, block_index: int
):
    module = _module()
    assert module is not None, "PID isolation entry point is missing"
    with pytest.raises(ValueError):
        module.command_for_block(case_id, block_index)


@pytest.mark.parametrize("case_id", ("unknown", ""))
def test_case_duration_rejects_unknown_case(case_id: str):
    module = _module()
    assert module is not None, "PID isolation entry point is missing"
    with pytest.raises(ValueError):
        module.case_duration_s(case_id)


def _constant_trace(samples: int = 100) -> dict[str, np.ndarray]:
    indices = np.arange(samples, dtype=np.int64)
    return {
        "raw_command_target": np.zeros((samples, 2), dtype=np.float32),
        "clipped_target": np.zeros((samples, 2), dtype=np.float32),
        "applied_action": np.zeros((samples, 2), dtype=np.float32),
        "joint_position_rad": np.zeros((samples, 3), dtype=np.float32),
        "joint_velocity_rad_s": np.zeros((samples, 3), dtype=np.float32),
        "roll_world_rad": np.zeros(samples, dtype=np.float64),
        "yaw_world_rad": np.zeros(samples, dtype=np.float64),
        "angular_velocity_world_rad_s": np.zeros((samples, 3), dtype=np.float64),
        "simulation_time_s": (indices + 1) * 0.02,
        "physics_step_index": indices + 1,
        "raw_simulation_time_s": 8.0 + (indices + 1) * 0.02,
        "raw_physics_step_index": 400 + indices + 1,
        "action_block_index": indices // 5,
        "physics_step_in_block": indices % 5 + 1,
    }


def test_constant_trace_has_zero_error_and_zero_tail_variation():
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    metrics = module.compute_isolation_metrics("hold_zero", _constant_trace())
    assert metrics["summary"]["aux_actual_applied_rms_rad"] == pytest.approx(0.0)
    assert metrics["summary"]["aux_actual_applied_max_abs_rad"] == pytest.approx(0.0)
    assert metrics["summary"]["aux_tail_peak_to_peak_rad"] == pytest.approx(0.0)
    assert metrics["summary"]["aux_tail_std_rad"] == pytest.approx(0.0)
    assert metrics["phases"][0]["overshoot_percent"] is None


def test_phase_reports_actual_main_axis_speed_summary():
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    trace = _constant_trace()
    trace["joint_velocity_rad_s"][:, 0] = 0.25
    metrics = module.compute_isolation_metrics("hold_zero", trace)
    speed = metrics["phases"][0]["main_actual_speed_rad_s"]
    assert speed["rms"] == pytest.approx(0.25)
    assert speed["max_abs"] == pytest.approx(0.25)


def test_timing_validation_rejects_a_real_world_clock_jump():
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    trace = _constant_trace(3)
    trace["simulation_time_s"][1] = 0.06
    with pytest.raises(ValueError, match="clock"):
        module.validate_physics_timing(trace)


def test_unstable_step_is_not_reported_as_zero_settling_time():
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    trace = _constant_trace(1500)
    target = math.pi / 18.0
    trace["raw_command_target"][:, 1] = target
    trace["clipped_target"][:, 1] = target
    trace["applied_action"][:, 1] = target
    trace["joint_position_rad"][:, 1] = np.where(
        np.arange(1500) % 2 == 0, target * 1.5, target * 0.5
    )
    metrics = module.compute_isolation_metrics("forward_pos10_return", trace)
    step_phase = next(item for item in metrics["phases"] if item["name"] == "forward_pos10")
    assert step_phase["settling_time_s"] is None
    assert step_phase["overshoot_percent"] == pytest.approx(50.0)


def test_case_directory_refuses_overwrite_and_preserves_partial(tmp_path):
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    partial, destination = module._atomic_case_directory(tmp_path, "hold_zero")
    assert partial.is_dir()
    partial.joinpath("failure.json").write_text("{}", encoding="utf-8")
    with pytest.raises(FileExistsError):
        module._atomic_case_directory(tmp_path, "hold_zero")
    assert partial.joinpath("failure.json").is_file()
    assert not destination.exists()


def test_runtime_error_result_stays_partial_and_never_publishes(tmp_path, monkeypatch):
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    project_root = Path(module.__file__).resolve().parents[1]
    scene_root = tmp_path / "scenes"
    scene_root.mkdir()
    fixture = scene_root / "tracking_open_plane.usda"
    fixture.write_bytes(b"cpu fake fixture")
    occupancy = tmp_path / "tracking_open_plane_free.npz"
    occupancy.write_bytes(b"cpu fake occupancy")
    contract = project_root / "configs" / "data_contract.yaml"
    drive_profile = project_root / "configs" / "benchmark_v1" / "drive_dynamics_v1.json"
    run_root = tmp_path / "runs"
    diagnostics = _constant_trace(6)
    diagnostics["position_world_xy"] = np.zeros((6, 2), dtype=np.float64)

    class FakeRuntime:
        def __init__(self, _config):
            self.physics_diagnostics = diagnostics
            self.captured_frames = {}
            self.close_return_code = None

        def set_reference_visualization_path(self, _reference):
            return None

        def close(self, *, return_code=1):
            self.close_return_code = return_code

    runtime_holder = []

    def runtime_factory(config):
        runtime = FakeRuntime(config)
        runtime_holder.append(runtime)
        return runtime

    failed_result = SimpleNamespace(
        termination_reason="runtime_error",
        runtime_error="RuntimeError: synthetic world failure on block 2",
        episode_id="pid_isolation_hold_zero",
    )

    def fake_run_episode(*_args, **_kwargs):
        return failed_result

    def fake_write_pid_artifacts(_output_root, _case_id, _diagnostics, **kwargs):
        partial = Path(kwargs["_partial_dir"])
        destination = Path(kwargs["_destination"])
        partial.joinpath("published-by-regression-stub").write_text("bad", encoding="utf-8")
        partial.replace(destination)
        return destination

    monkeypatch.setattr(module, "_build_episode", lambda *args, **kwargs: SimpleNamespace(
        episode_id="pid_isolation_hold_zero"
    ))
    import benchmark.runner

    monkeypatch.setattr(benchmark.runner, "run_episode", fake_run_episode)
    monkeypatch.setattr(module, "write_pid_artifacts", fake_write_pid_artifacts)

    with pytest.raises(RuntimeError, match="synthetic world failure"):
        module.run_pid_case(
            "hold_zero",
            run_root=run_root,
            scene_root=scene_root,
            fixture=fixture,
            occupancy=occupancy,
            contract=contract,
            drive_profile=drive_profile,
            runtime_factory=runtime_factory,
        )

    partial = run_root / ".hold_zero.partial"
    destination = run_root / "hold_zero"
    assert partial.is_dir()
    assert not destination.exists()
    failure = json.loads(partial.joinpath("failure.json").read_text(encoding="utf-8"))
    assert "synthetic world failure" in failure["error"]
    assert "synthetic world failure" in partial.joinpath("exception.txt").read_text(
        encoding="utf-8"
    )
    assert runtime_holder[0].close_return_code == 1


def test_cli_requires_one_case_and_defaults_to_headless_baseline_gains(tmp_path):
    module = _module()
    assert module is not None, "PID isolation entry point is missing"

    args = module._build_parser().parse_args(
        ["--case-id", "hold_zero", "--run-root", str(tmp_path)]
    )
    assert args.case_id == "hold_zero"
    assert args.run_root == tmp_path
    assert args.gui is False
    assert args.aux_kp == pytest.approx(300.0)
    assert args.aux_kd == pytest.approx(60.0)
    with pytest.raises(SystemExit):
        module._build_parser().parse_args(["--run-root", str(tmp_path)])
