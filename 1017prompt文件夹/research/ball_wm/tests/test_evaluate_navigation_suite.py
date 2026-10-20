"""Isaac-free tests for strict multi-scene PointGoal evaluation aggregation."""

from __future__ import annotations

import pytest

try:
    from training.evaluate_navigation_suite import (
        assert_policy_contexts_match,
        context_tuples,
        scene_seed,
        summarize_suite_policy,
    )
except ImportError:
    assert_policy_contexts_match = None
    context_tuples = None
    scene_seed = None
    summarize_suite_policy = None

try:
    from training.evaluate_navigation_suite import build_local_policy_cases
except ImportError:
    build_local_policy_cases = None


def _episode(
    *,
    episode_index: int,
    scene_path: str,
    reason: str,
    final_distance_m: float,
) -> dict[str, object]:
    return {
        "episode_index": episode_index,
        "context_start": 3,
        "context_last_frame": 7,
        "goal_frame": 27,
        "scene_path": scene_path,
        "termination_reason": reason,
        "collision": reason == "collision",
        "steps": 4,
        "final_distance_m": final_distance_m,
        "min_clearance_m": 1.0,
        "reference_length_m": 1.0,
        "path_length_m": 1.0,
        "trajectory_xy": [[0.0, 0.0], [1.0, 0.0]],
        "runtime_error": None,
    }


def _scene_result(episode: dict[str, object]) -> dict[str, object]:
    return {"episodes": [episode], "runtime_error_count": 0}


def test_scene_seed_is_stable_and_changes_between_scenes():
    """Catches a suite that makes policy comparisons non-reproducible by scene."""

    assert scene_seed(3072, 0) == 3072
    assert scene_seed(3072, 1) == 3073


def test_suite_summary_uses_all_raw_scene_episode_records():
    """Catches an aggregate that reports only the final scene's success rate."""

    results = [
        _scene_result(
            _episode(
                episode_index=0,
                scene_path="/assets/scene_easy_0/scene.usd",
                reason="success",
                final_distance_m=0.1,
            )
        ),
        _scene_result(
            _episode(
                episode_index=1,
                scene_path="/assets/scene_easy_1/scene.usd",
                reason="timeout",
                final_distance_m=1.0,
            )
        ),
    ]

    summary = summarize_suite_policy(results, 0.2, 50)

    assert summary["num_eval"] == 2
    assert summary["success_rate"] == pytest.approx(0.5)
    assert summary["timeout_rate"] == pytest.approx(0.5)
    assert summary["runtime_error_count"] == 0


def test_suite_rejects_nonidentical_policy_contexts():
    """Catches a policy evaluated on a different target/context selection."""

    baseline = _scene_result(
        _episode(
            episode_index=0,
            scene_path="/assets/scene_easy_0/scene.usd",
            reason="timeout",
            final_distance_m=1.0,
        )
    )
    shifted_spatial_episode = dict(baseline["episodes"][0])
    shifted_spatial_episode["goal_frame"] = 28
    spatial = _scene_result(shifted_spatial_episode)

    with pytest.raises(ValueError, match="identical context"):
        assert_policy_contexts_match({"baseline": [baseline], "spatial": [spatial]})


def test_context_tuples_are_sorted_by_scene_and_episode():
    """Catches order-dependent comparison of otherwise identical scene results."""

    result = {
        "episodes": [
            _episode(
                episode_index=2,
                scene_path="/assets/scene_easy_1/scene.usd",
                reason="timeout",
                final_distance_m=1.0,
            ),
            _episode(
                episode_index=1,
                scene_path="/assets/scene_easy_0/scene.usd",
                reason="timeout",
                final_distance_m=1.0,
            ),
        ]
    }

    tuples = context_tuples(result)

    assert tuples == [
        (1, 3, 7, 27, "/assets/scene_easy_0/scene.usd"),
        (2, 3, 7, 27, "/assets/scene_easy_1/scene.usd"),
    ]


def test_suite_constructs_pose_only_and_pose_bev_cases_from_one_checkpoint(tmp_path):
    """Catches a formal comparison that quietly uses different checkpoints or planner modes."""

    assert build_local_policy_cases is not None, "local PointGoal policy cases are not available"
    cases = {case.name: case for case in build_local_policy_cases(tmp_path / "pose.ckpt", safety_weight=0.25)}

    assert set(cases) == {"pose_only_cem", "pose_bev_cem", "random"}
    assert cases["pose_only_cem"].planner_mode == "pose_bev"
    assert cases["pose_bev_cem"].planner_mode == "pose_bev"
    assert cases["pose_only_cem"].safety_weight == pytest.approx(0.0)
    assert cases["pose_bev_cem"].safety_weight == pytest.approx(0.25)
    assert cases["random"].policy == "random"
