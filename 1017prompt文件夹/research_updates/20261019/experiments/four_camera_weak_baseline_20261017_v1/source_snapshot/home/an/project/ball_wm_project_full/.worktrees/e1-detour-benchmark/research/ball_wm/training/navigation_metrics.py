"""Pure metrics and action-shape helpers for closed-loop PointGoal evaluation."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import Callable

import numpy as np


TERMINATION_REASONS = ("success", "collision", "timeout", "runtime_error")


def _average_ranks(values: Sequence[float]) -> np.ndarray:
    """Return one-indexed average ranks, assigning tied values their shared mean rank."""

    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or len(array) < 2 or not np.isfinite(array).all():
        raise ValueError("rank inputs must contain at least two finite scalar values")
    order = np.argsort(array, kind="stable")
    sorted_values = array[order]
    ranks = np.empty(len(array), dtype=np.float64)
    start = 0
    while start < len(array):
        end = start + 1
        while end < len(array) and sorted_values[end] == sorted_values[start]:
            end += 1
        ranks[order[start:end]] = (start + 1 + end) / 2.0
        start = end
    return ranks


def spearman_rank_correlation(predicted_costs: Sequence[float], actual_costs: Sequence[float]) -> float:
    """Return a finite Spearman rank correlation, treating constant rankings as uninformative."""

    if len(predicted_costs) != len(actual_costs):
        raise ValueError("predicted_costs and actual_costs must have equal length")
    predicted_ranks = _average_ranks(predicted_costs)
    actual_ranks = _average_ranks(actual_costs)
    predicted_centered = predicted_ranks - predicted_ranks.mean()
    actual_centered = actual_ranks - actual_ranks.mean()
    denominator = float(np.linalg.norm(predicted_centered) * np.linalg.norm(actual_centered))
    if denominator <= np.finfo(np.float64).eps:
        return 0.0
    return float(np.clip(np.dot(predicted_centered, actual_centered) / denominator, -1.0, 1.0))


def bootstrap_interval(
    values: Sequence[float],
    statistic: Callable[[np.ndarray], float],
    *,
    seed: int,
    samples: int = 1000,
    confidence: float = 0.95,
) -> dict[str, float | int]:
    """Return a seeded percentile bootstrap interval over independent raw episode values."""

    data = np.asarray(values, dtype=np.float64)
    if data.ndim != 1 or len(data) < 1 or not np.isfinite(data).all():
        raise ValueError("bootstrap values must be a non-empty finite one-dimensional sequence")
    if samples < 1 or not math.isfinite(float(confidence)) or not 0.0 < float(confidence) < 1.0:
        raise ValueError("bootstrap samples must be positive and confidence must lie in (0, 1)")
    estimate = float(statistic(data))
    if not math.isfinite(estimate):
        raise ValueError("bootstrap statistic must return a finite estimate")
    rng = np.random.default_rng(seed)
    resampled = np.empty(samples, dtype=np.float64)
    for index in range(samples):
        resampled[index] = float(statistic(data[rng.integers(0, len(data), size=len(data))]))
    if not np.isfinite(resampled).all():
        raise ValueError("bootstrap statistic must return finite resampled estimates")
    alpha = (1.0 - float(confidence)) / 2.0
    return {
        "estimate": estimate,
        "low": float(np.quantile(resampled, alpha)),
        "high": float(np.quantile(resampled, 1.0 - alpha)),
        "confidence": float(confidence),
        "bootstrap_samples": int(samples),
    }


def cluster_bootstrap_interval(
    episodes: Sequence[Mapping[str, object]],
    *,
    field: str,
    cluster_field: str,
    seed: int,
    samples: int = 1000,
    confidence: float = 0.95,
) -> dict[str, float | int]:
    """Bootstrap raw episodes by resampling whole scene clusters."""

    if not episodes:
        raise ValueError("cluster bootstrap requires at least one episode")
    if not isinstance(field, str) or not field or not isinstance(cluster_field, str) or not cluster_field:
        raise ValueError("field and cluster_field must be non-empty strings")
    if samples < 1 or not math.isfinite(float(confidence)) or not 0.0 < float(confidence) < 1.0:
        raise ValueError("bootstrap samples must be positive and confidence must lie in (0, 1)")

    clusters: dict[object, list[float]] = {}
    all_values: list[float] = []
    for episode in episodes:
        if cluster_field not in episode:
            raise ValueError(f"episode is missing cluster field {cluster_field!r}")
        cluster = episode[cluster_field]
        try:
            hash(cluster)
        except TypeError as error:
            raise ValueError(f"cluster field {cluster_field!r} must be hashable") from error
        if cluster is None:
            raise ValueError(f"cluster field {cluster_field!r} cannot be None")
        if field not in episode:
            raise ValueError(f"episode is missing metric field {field!r}")
        value = float(episode[field])
        if not math.isfinite(value):
            raise ValueError(f"metric field {field!r} must contain finite values")
        clusters.setdefault(cluster, []).append(value)
        all_values.append(value)

    cluster_values = [np.asarray(values, dtype=np.float64) for values in clusters.values()]
    rng = np.random.default_rng(seed)
    resampled = np.empty(samples, dtype=np.float64)
    for sample_index in range(samples):
        chosen = rng.integers(0, len(cluster_values), size=len(cluster_values))
        sample = np.concatenate([cluster_values[int(index)] for index in chosen])
        resampled[sample_index] = float(np.mean(sample))
    alpha = (1.0 - float(confidence)) / 2.0
    return {
        "estimate": float(np.mean(np.asarray(all_values, dtype=np.float64))),
        "low": float(np.quantile(resampled, alpha)),
        "high": float(np.quantile(resampled, 1.0 - alpha)),
        "confidence": float(confidence),
        "bootstrap_samples": int(samples),
        "num_clusters": len(cluster_values),
    }


def action_block_to_controls(
    action_block: np.ndarray,
    action_dim: int = 10,
    controls_per_block: int = 5,
) -> np.ndarray:
    """Reshape one model action block into five physical two-DOF controls."""

    values = np.asarray(action_block, dtype=np.float32)
    if action_dim != controls_per_block * 2 or values.shape != (action_dim,):
        raise ValueError(f"action block must have shape ({action_dim},)")
    if not np.isfinite(values).all():
        raise ValueError("action block must contain only finite values")
    return values.reshape(controls_per_block, 2).copy()


def episode_termination(
    distance_m: float,
    collision: bool,
    step: int,
    max_steps: int,
    goal_tolerance_m: float,
) -> str | None:
    """Return the terminal reason with collision taking precedence."""

    if not math.isfinite(float(distance_m)) or float(distance_m) < 0.0:
        raise ValueError("distance_m must be finite and non-negative")
    if int(step) < 0 or int(max_steps) < 1:
        raise ValueError("step must be non-negative and max_steps must be positive")
    if not math.isfinite(float(goal_tolerance_m)) or float(goal_tolerance_m) <= 0.0:
        raise ValueError("goal_tolerance_m must be finite and positive")
    if bool(collision):
        return "collision"
    if float(distance_m) <= float(goal_tolerance_m):
        return "success"
    if int(step) >= int(max_steps):
        return "timeout"
    return None


def path_length(points_xy: np.ndarray) -> float:
    """Return the Euclidean length of a finite XY polyline."""

    points = np.asarray(points_xy, dtype=np.float64)
    if points.size == 0:
        return 0.0
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("points_xy must be a finite array with shape (N, 2)")
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def success_weighted_path_length(
    success: bool,
    path_length_m: float,
    reference_length_m: float,
) -> float:
    """Compute the zero-for-failure SPL contribution of one episode."""

    if not success:
        return 0.0
    path = float(path_length_m)
    reference = float(reference_length_m)
    if not math.isfinite(path) or not math.isfinite(reference) or path < 0.0 or reference < 0.0:
        raise ValueError("path_length_m and reference_length_m must be finite and non-negative")
    if reference == 0.0:
        return 0.0
    return reference / max(path, reference, np.finfo(np.float64).eps)


def _finite_mean(episodes: Sequence[Mapping[str, object]], field: str) -> float:
    values: list[float] = []
    for episode in episodes:
        value = episode.get(field)
        if value is None:
            continue
        number = float(value)
        if math.isfinite(number):
            values.append(number)
    return float(np.mean(values)) if values else 0.0


def summarize_navigation(
    episodes: Sequence[Mapping[str, object]],
    goal_tolerance_m: float,
    max_mpc_steps: int,
) -> dict[str, object]:
    """Summarize mutually exclusive terminal outcomes and trajectory metrics."""

    if not math.isfinite(float(goal_tolerance_m)) or float(goal_tolerance_m) <= 0.0:
        raise ValueError("goal_tolerance_m must be finite and positive")
    if int(max_mpc_steps) < 1:
        raise ValueError("max_mpc_steps must be positive")

    total = len(episodes)
    counts = {reason: 0 for reason in TERMINATION_REASONS}
    spl_values: list[float] = []
    for episode in episodes:
        reason = episode.get("termination_reason")
        if reason not in counts:
            raise ValueError(f"unknown termination_reason: {reason!r}")
        counts[str(reason)] += 1
        spl_values.append(
            success_weighted_path_length(
                reason == "success",
                float(episode.get("path_length_m") or 0.0),
                float(episode.get("reference_length_m") or 0.0),
            )
        )

    denominator = max(1, total)
    return {
        "num_eval": total,
        "goal_tolerance_m": float(goal_tolerance_m),
        "max_mpc_steps": int(max_mpc_steps),
        "success_rate": counts["success"] / denominator,
        "collision_rate": counts["collision"] / denominator,
        "timeout_rate": counts["timeout"] / denominator,
        "runtime_error_rate": counts["runtime_error"] / denominator,
        "runtime_error_count": counts["runtime_error"],
        "final_distance_mean_m": _finite_mean(episodes, "final_distance_m"),
        "path_length_mean_m": _finite_mean(episodes, "path_length_m"),
        "spl": float(np.mean(spl_values)) if spl_values else 0.0,
    }
