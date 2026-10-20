import sys
from pathlib import Path

import torch
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import training.planner as planner
from training.action_protocol import ActionProtocol
from training.joint_jepa import JointJEPA, _ProbeHead
from training.lewm_modules import ARPredictor, Embedder, MLP
from training.planner import latent_goal_cost, plan


def _aux30_protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )


def _tiny_model() -> JointJEPA:
    config_embed = 16
    encoder = torch.nn.Sequential(
        torch.nn.Flatten(),
        torch.nn.Linear(6 * 8 * 8, config_embed),
    )
    predictor = ARPredictor(
        num_frames=3,
        depth=1,
        heads=2,
        mlp_dim=32,
        input_dim=config_embed,
        hidden_dim=config_embed,
        output_dim=config_embed,
        dim_head=8,
    )
    model = JointJEPA(
        encoder=encoder,
        predictor=predictor,
        action_encoder=Embedder(10, emb_dim=config_embed),
        proprio_encoder=torch.nn.Linear(17, config_embed),
        bev_head=_ProbeHead(config_embed, 3, 16),
        current_head=_ProbeHead(config_embed, 1, 16),
        history_head=_ProbeHead(config_embed, 1, 16),
        projector=MLP(config_embed, 32, config_embed),
        pred_proj=MLP(config_embed, 32, config_embed),
    )
    model.num_preds = 1
    model.history_size = 3
    return model


def test_latent_goal_cost_shape():
    predicted = torch.randn(2, 4, 3, 16)
    goal = torch.randn(2, 1, 3, 16)
    cost = latent_goal_cost(predicted, goal)
    assert cost.shape == (2, 4)


def test_action_bounds_repeat_main_and_aux_ranges_per_control_pair():
    """Each flattened five-control block must retain main/aux physical ranges."""

    bounds = planner.action_bounds(_aux30_protocol())

    expected = torch.tensor(
        [[-2.0, 2.0], [-0.5235987756, 0.5235987756]] * 5,
        dtype=torch.float32,
    )
    assert torch.allclose(bounds, expected)


def test_plan_returns_bounded_action_sequence():
    model = _tiny_model()
    context = {
        "pixels_left": torch.randn(1, 3, 3, 8, 8),
        "pixels_right": torch.randn(1, 3, 3, 8, 8),
        "action": torch.randn(1, 3, 10),
        "proprio": torch.randn(1, 3, 17),
    }
    goal_emb = torch.randn(1, 1, 16)
    result = plan(
        model,
        context,
        action_protocol=_aux30_protocol(),
        goal_emb=goal_emb,
        horizon=4,
        samples=8,
        elites=3,
        iterations=2,
        safety_weight=0.0,
        history_size=3,
        device=torch.device("cpu"),
    )
    assert result.actions.shape == (4, 10)
    assert result.cost == result.cost  # not NaN
    assert torch.isfinite(result.actions).all()
    bounds = planner.action_bounds(_aux30_protocol())
    assert (result.actions >= bounds[:, 0] - 1e-5).all()
    assert (result.actions <= bounds[:, 1] + 1e-5).all()


class _ZeroAnchorPoseModel:
    pose_head = object()

    def rollout(self, context, action_sequence, history_size):
        del context, history_size
        pose = torch.zeros(*action_sequence.shape[:-1], 4, device=action_sequence.device)
        pose[..., :2] = action_sequence[..., :2]
        pose[..., 3] = 1.0
        return pose

    def decode_relative_pose(self, future_embeddings):
        return future_embeddings


def test_cem_keeps_zero_anchor_when_all_sampled_actions_are_worse():
    """CEM must not return a sampled action with higher cost than neutral motion."""

    result = plan(
        _ZeroAnchorPoseModel(),
        {
            "pixels_left": torch.zeros(1, 3, 3, 8, 8),
            "pixels_right": torch.zeros(1, 3, 3, 8, 8),
            "action": torch.zeros(1, 3, 10),
            "proprio": torch.zeros(1, 3, 17),
        },
        action_protocol=_aux30_protocol(),
        goal_emb=None,
        goal_relative_xy=torch.zeros(1, 2),
        planner_mode="pose_bev",
        horizon=2,
        samples=8,
        elites=2,
        iterations=1,
        safety_weight=0.0,
        history_size=3,
        device=torch.device("cpu"),
        sigma=1.0,
    )

    assert result.cost == pytest.approx(0.0)
    assert torch.allclose(result.actions, torch.zeros_like(result.actions))


def test_cem_keeps_sampling_finite_when_only_one_elite_is_safe(monkeypatch):
    """A single safe elite must not make the next CEM scale NaN."""

    calls = 0

    def fake_score_action_sequences(model, context, action_sequences, **kwargs):
        del model, context, kwargs
        nonlocal calls
        calls += 1
        assert torch.isfinite(action_sequences).all()
        samples = action_sequences.size(1)
        costs = torch.arange(samples, dtype=action_sequences.dtype).reshape(1, samples)
        safety = torch.ones_like(costs)
        safety[:, 0] = 0.0
        zeros = torch.zeros_like(costs)
        return planner.CandidateScores(
            total_costs=costs,
            goal_costs=costs,
            safety_costs=safety,
            smoothness_costs=zeros,
            predicted_relative_pose=None,
        )

    monkeypatch.setattr(planner, "score_action_sequences", fake_score_action_sequences)
    result = plan(
        object(),
        {"action": torch.zeros(1, 3, 10)},
        action_protocol=_aux30_protocol(),
        goal_emb=None,
        goal_relative_xy=torch.zeros(1, 2),
        planner_mode="pose_bev",
        horizon=2,
        samples=4,
        elites=2,
        iterations=2,
        safety_weight=0.1,
        safety_threshold=0.0,
        history_size=3,
        device=torch.device("cpu"),
    )

    assert calls == 2
    assert torch.isfinite(result.actions).all()


class _PrimitiveAnchorPoseModel:
    pose_head = object()

    def rollout(self, context, action_sequence, history_size):
        del context, history_size
        # The synthetic model has a unique optimum at the persistent
        # forward-plus-turn primitive (main=1.4, aux=0.366519).  A Gaussian
        # population should not hit this exact structured sequence by chance.
        main = action_sequence[..., 0].mean(dim=-1)
        aux = action_sequence[..., 1].mean(dim=-1)
        endpoint = ((main - 1.4).square() + (aux - 0.36651915).square()).sqrt()
        pose = torch.zeros(*action_sequence.shape[:-1], 4, device=action_sequence.device)
        pose[..., -1, 0] = endpoint
        pose[..., 3] = 1.0
        return pose

    def decode_relative_pose(self, future_embeddings):
        return future_embeddings


def test_cem_includes_structured_primitive_anchor():
    """CEM must score persistent forward/turn primitives, not only Gaussian noise."""

    result = plan(
        _PrimitiveAnchorPoseModel(),
        {
            "pixels_left": torch.zeros(1, 3, 3, 8, 8),
            "pixels_right": torch.zeros(1, 3, 3, 8, 8),
            "action": torch.zeros(1, 3, 10),
            "proprio": torch.zeros(1, 3, 17),
        },
        action_protocol=_aux30_protocol(),
        goal_emb=None,
        goal_relative_xy=torch.zeros(1, 2),
        planner_mode="pose_bev",
        horizon=5,
        samples=8,
        elites=2,
        iterations=1,
        safety_weight=0.0,
        history_size=3,
        device=torch.device("cpu"),
        sigma=0.01,
    )

    assert result.cost == pytest.approx(0.0, abs=1e-6)
    expected = torch.zeros(5, 10)
    expected[:, 0::2] = 1.4
    expected[:, 1::2] = 0.36651915
    assert torch.allclose(result.actions, expected, atol=1e-6)


def test_plan_reports_goal_and_safety_components_of_selected_cost():
    """Catches a diagnostic planner result that hides an unsafe cost tradeoff."""

    torch.manual_seed(7)
    model = _tiny_model()
    context = {
        "pixels_left": torch.randn(1, 3, 3, 8, 8),
        "pixels_right": torch.randn(1, 3, 3, 8, 8),
        "action": torch.randn(1, 3, 10),
        "proprio": torch.randn(1, 3, 17),
    }
    goal_emb = torch.randn(1, 1, 16)

    result = plan(
        model,
        context,
        action_protocol=_aux30_protocol(),
        goal_emb=goal_emb,
        horizon=3,
        samples=6,
        elites=2,
        iterations=2,
        safety_weight=0.25,
        history_size=3,
        device=torch.device("cpu"),
    )

    assert result.goal_cost >= 0.0
    assert result.safety_cost >= 0.0
    assert result.cost == pytest.approx(result.goal_cost + 0.25 * result.safety_cost)


def _occupied_logits_at(row: int, col: int, size: int) -> torch.Tensor:
    logits = torch.full((1, 3, size, size), -10.0)
    logits[:, 0] = 10.0
    logits[:, 1, row, col] = 10.0
    logits[:, 0, row, col] = -10.0
    return logits


def test_relative_pose_goal_cost_prefers_candidate_with_matching_base_endpoint():
    """Catches a goal cost that reads an intermediate pose or reverses candidate ordering."""

    cost_fn = getattr(planner, "relative_pose_goal_cost", None)
    if cost_fn is None:
        pytest.fail("relative_pose_goal_cost is not available")
    predicted = torch.tensor(
        [[[[0.1, 0.0, 0.0, 1.0]], [[1.0, 0.0, 0.0, 1.0]]]],
        dtype=torch.float32,
    )

    cost = cost_fn(predicted, torch.tensor([[1.0, 0.0]], dtype=torch.float32))

    assert cost.shape == (1, 2)
    assert cost[0, 1] < cost[0, 0]


def test_local_bev_safety_maps_forward_to_smaller_row_and_marks_out_of_bounds_unsafe():
    """Catches a BEV axis flip or a trajectory outside visual range treated as free space."""

    safety_fn = getattr(planner, "local_bev_trajectory_safety_cost", None)
    if safety_fn is None:
        pytest.fail("local_bev_trajectory_safety_cost is not available")
    logits = _occupied_logits_at(row=3, col=5, size=11)
    poses = torch.tensor(
        [[[[2.0, 0.0, 0.0, 1.0]], [[20.0, 0.0, 0.0, 1.0]]]],
        dtype=torch.float32,
    )

    safety = safety_fn(
        logits,
        poses,
        resolution_m=1.0,
        robot_radius_m=0.0,
        unknown_weight=1.0,
    )

    assert safety.shape == (1, 2)
    assert safety[0, 0] > 0.9
    assert safety[0, 1] > 0.9


def test_action_smoothness_cost_penalizes_oscillation_but_not_constant_blocks():
    """Catches a smoothness term that penalizes constant control or ignores abrupt changes."""

    smoothness_fn = getattr(planner, "action_smoothness_cost", None)
    if smoothness_fn is None:
        pytest.fail("action_smoothness_cost is not available")
    actions = torch.zeros(2, 3, 10)
    actions[1, 1] = 1.0

    cost = smoothness_fn(actions)

    assert cost.shape == (2,)
    assert cost[0] == pytest.approx(0.0)
    assert cost[1] > 0.0


class _AllUnsafePoseModel:
    pose_head = object()

    def rollout(self, context, action_sequence, history_size):
        batch, samples, horizon, _ = action_sequence.shape
        prediction = torch.zeros(batch, samples, horizon, 4)
        prediction[..., 3] = 1.0
        return prediction

    def decode_relative_pose(self, future_embeddings):
        return future_embeddings

    def encode(self, context):
        batch, frames = context["action"].shape[:2]
        return {"emb": torch.zeros(batch, frames, 4)}

    @staticmethod
    def bev_head(embeddings):
        logits = torch.full((*embeddings.shape[:-1], 3, 11, 11), -10.0)
        logits[..., 1, :, :] = 10.0
        return logits


def test_pose_bev_plan_stops_when_every_candidate_is_unsafe():
    """Catches CEM choosing the least-bad collision trajectory instead of the specified safe stop fallback."""

    result = plan(
        _AllUnsafePoseModel(),
        {
            "pixels_left": torch.zeros(1, 3, 3, 8, 8),
            "pixels_right": torch.zeros(1, 3, 3, 8, 8),
            "action": torch.zeros(1, 3, 10),
            "proprio": torch.zeros(1, 3, 17),
        },
        action_protocol=_aux30_protocol(),
        goal_emb=None,
        goal_relative_xy=torch.tensor([[1.0, 0.0]]),
        planner_mode="pose_bev",
        horizon=3,
        samples=4,
        elites=2,
        iterations=1,
        safety_weight=1.0,
        safety_threshold=0.2,
        history_size=3,
        device=torch.device("cpu"),
    )

    assert result.fallback_used is True
    assert torch.count_nonzero(result.actions) == 0
