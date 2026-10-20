"""The second eye must be observed without changing the frozen controller."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from benchmark.policies import EpisodeContext, PolicyObservation
from training.action_protocol import ActionProtocol

from bilateral_shadow import BilateralShadowPolicy
from policy_continuous import ContinuousVisionDetourPolicy


PILOT = Path(__file__).resolve().parent.parent


def observation(left: np.ndarray, right: np.ndarray) -> PolicyObservation:
    return PolicyObservation(
        stereo_left_rgb=left,
        stereo_right_rgb=right,
        proprio_history=np.zeros((1, 17), dtype=np.float32),
        action_history=np.zeros((1, 10), dtype=np.float32),
        position_world_xy=np.array([-3.3, 0.0]),
        yaw_world_rad=0.0,
    )


def test_right_eye_changes_shadow_evidence_but_never_actions() -> None:
    with np.load(PILOT / "integrated_fill_e1/depth_truth/frame_000.npz") as e1:
        left = e1["left_rgb"].copy()
        e1_right = e1["right_rgb"].copy()
    with np.load(PILOT / "integrated_fill_e2/depth_truth/frame_000.npz") as e2:
        e2_right = e2["right_rgb"].copy()

    protocol = ActionProtocol("ball_action_v2_aux30", (-2.0, -np.pi / 6),
                              (2.0, np.pi / 6), 5)
    context = EpisodeContext("test_e1", False, None, protocol.protocol_id)
    baseline = ContinuousVisionDetourPolicy(protocol)
    shadow_e1 = BilateralShadowPolicy(protocol)
    shadow_e2 = BilateralShadowPolicy(protocol)
    for policy in (baseline, shadow_e1, shadow_e2):
        policy.reset(context)

    for _ in range(2):
        goal = np.array([6.6, 0.0])
        baseline_action = baseline.act(observation(left, e1_right), goal)
        e1_action = shadow_e1.act(observation(left, e1_right), goal)
        e2_action = shadow_e2.act(observation(left, e2_right), goal)
        assert np.array_equal(e1_action, baseline_action)
        assert np.array_equal(e2_action, baseline_action)

    assert shadow_e1.records[-1]["detour_triggered"]
    assert shadow_e2.records[-1]["detour_triggered"]
    assert shadow_e1.records[-1]["right_obstacle_pixels"] != shadow_e2.records[-1]["right_obstacle_pixels"]
    assert shadow_e1.records[-1]["right_obstacle_pixels"] > 0
