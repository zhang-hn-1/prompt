"""Focused observability tests for failed formal benchmark episodes."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def test_run_one_persists_failure_audit_before_close_masks_original_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    from scripts import run_pointgoal_benchmark as entrypoint

    manifest = tmp_path / "manifest.json"
    manifest.write_text('{"fixture": true}\n', encoding="utf-8")
    args = SimpleNamespace(
        run_root=tmp_path / "run",
        manifest=manifest,
        contract=tmp_path / "contract.yaml",
        scene_index=tmp_path / "scene_index.json",
        scene_root=tmp_path / "scenes",
        data_root=None,
        gui=False,
        policy="oracle_v2",
        seed=3072,
        oracle_lookahead_m=0.5,
    )
    episode = SimpleNamespace(episode_id="hard_0_e1_0001", seed=20260912)
    plan = SimpleNamespace(status="timeout", expanded_nodes=17, duration_s=31.5)
    result = SimpleNamespace(
        positions_world_xy=np.zeros((1, 2), dtype=np.float64),
        runtime_error="runtime reported an inner failure",
        termination_reason="runtime_error",
    )
    events: list[str] = []

    class FakeRuntime:
        def __init__(self, _config: object) -> None:
            self.dynamic_plan = self._dynamic_plan

        def _dynamic_plan(self, _episode: object) -> object:
            events.append("plan")
            return plan

        def close(self, return_code: int = 0) -> None:
            events.append(f"close:{return_code}")
            raise RuntimeError("simulation app shutting down")

    monkeypatch.setattr(
        entrypoint,
        "load_action_protocol",
        lambda _path: SimpleNamespace(protocol_id="ball_action_v2_aux30"),
    )
    monkeypatch.setattr(
        entrypoint,
        "_resolve_occupancy_path",
        lambda *_args: tmp_path / "occupancy.npy",
    )
    monkeypatch.setattr(entrypoint, "IsaacBenchmarkRuntime", FakeRuntime)
    monkeypatch.setattr(entrypoint, "_make_policy", lambda *_args, **_kwargs: object())

    def run_episode_that_records_plan(runtime: object, *_args, **_kwargs) -> object:
        runtime.dynamic_plan(episode)
        return result

    monkeypatch.setattr(entrypoint, "run_episode", run_episode_that_records_plan)

    def render_that_fails(*_args, **_kwargs) -> None:
        raise ValueError("trajectory renderer failed")

    monkeypatch.setattr(entrypoint, "_render_task_and_trajectory", render_that_fails)

    with pytest.raises(ValueError, match="trajectory renderer failed"):
        entrypoint._run_one(args, episode)

    partial = args.run_root / "episode_hard_0_e1_0001.partial"
    audit = json.loads((partial / "failure.json").read_text(encoding="utf-8"))
    assert audit["exception_type"] == "ValueError"
    assert audit["exception_message"] == "trajectory renderer failed"
    assert audit["result"] == {
        "runtime_error": "runtime reported an inner failure",
        "termination_reason": "runtime_error",
    }
    assert audit["planning_audit"] == {
        "status": "timeout",
        "expanded_nodes": 17,
        "duration_s": 31.5,
        "planning_wall_time_s": pytest.approx(0.0, abs=1.0),
    }
    assert events == ["plan", "close:1"]
    assert "trajectory renderer failed" in capsys.readouterr().err
    assert not (args.run_root / "episode_hard_0_e1_0001").exists()
