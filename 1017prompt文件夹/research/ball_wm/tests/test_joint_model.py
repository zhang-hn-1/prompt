from dataclasses import asdict
import sys
import types
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import training.joint_jepa as joint_jepa
from training.joint_jepa import (
    JointModelConfig,
    JointJEPA,
    _ProbeHead,
    _occupied_dice_loss,
    _bev_cross_entropy,
    SpatialProbeHead,
    build_joint_model,
)
from training.lewm_modules import ARPredictor, Embedder, MLP, SIGReg


def _dummy_config() -> JointModelConfig:
    return JointModelConfig(
        image_size=32,
        embed_dim=32,
        action_dim=10,
        proprio_dim=17,
        bev_classes=3,
        bev_size=16,
        history_size=3,
        num_preds=1,
    )


class _FakeEncoder(torch.nn.Module):
    def __init__(self, embed_dim: int):
        super().__init__()
        self.embed_dim = embed_dim

    def forward(self, pixels: torch.Tensor) -> torch.Tensor:
        return torch.randn(pixels.size(0), self.embed_dim)


def test_joint_model_forward_shapes_and_backward():
    config = _dummy_config()
    encoder = _FakeEncoder(config.embed_dim)
    predictor = ARPredictor(
        num_frames=config.history_size,
        depth=2,
        heads=4,
        mlp_dim=64,
        input_dim=config.embed_dim,
        hidden_dim=config.embed_dim,
        output_dim=config.embed_dim,
        dim_head=16,
    )
    model = JointJEPA(
        encoder=encoder,
        predictor=predictor,
        action_encoder=Embedder(config.action_dim, emb_dim=config.embed_dim),
        proprio_encoder=torch.nn.Sequential(
            torch.nn.Linear(config.proprio_dim, config.embed_dim),
            torch.nn.SiLU(),
            torch.nn.Linear(config.embed_dim, config.embed_dim),
        ),
        bev_head=_ProbeHead(config.embed_dim, 3, 16),
        current_head=_ProbeHead(config.embed_dim, 1, 16),
        history_head=_ProbeHead(config.embed_dim, 1, 16),
        projector=MLP(config.embed_dim, 64, config.embed_dim),
        pred_proj=MLP(config.embed_dim, 64, config.embed_dim),
    )
    model.num_preds = config.num_preds
    model.history_size = config.history_size

    info = {
        "pixels_left": torch.randn(2, 4, 3, 32, 32),
        "pixels_right": torch.randn(2, 4, 3, 32, 32),
        "action": torch.randn(2, 4, 10),
        "proprio": torch.randn(2, 4, 17),
        "geometry_bev": torch.randint(0, 3, (2, 4, 16, 16)),
        "current_visible": torch.randint(0, 2, (2, 4, 16, 16)).float(),
        "history_observed": torch.randint(0, 2, (2, 4, 16, 16)).float(),
    }

    output = model(info)
    assert output["emb"].shape == (2, 4, 32)
    assert output["act_emb"].shape == (2, 4, 32)
    assert output["pred_emb"].shape == (2, 3, 32)

    losses = model.losses(output, SIGReg(knots=5, num_proj=32))
    assert losses["loss"].ndim == 0
    losses["loss"].backward()
    assert any(p.grad is not None for p in model.parameters())


def test_build_joint_model_instantiates_timm_encoder():
    config = _dummy_config()
    model = build_joint_model(config)
    assert isinstance(model.encoder, torch.nn.Module)
    assert model.num_preds == 1


def test_bev_class_weight_increases_penalty_for_missed_occupied_cells():
    logits = torch.tensor([[[[4.0, 4.0]], [[0.0, 0.0]], [[0.0, 0.0]]]])
    target = torch.tensor([[[0, 1]]])

    unweighted = _bev_cross_entropy(logits, target, class_weights=None)
    weighted = _bev_cross_entropy(logits, target, class_weights=(1.0, 8.0, 1.0))

    assert weighted > unweighted


def test_visibility_weight_increases_penalty_in_observed_cells():
    logits = torch.tensor([[[[4.0, 4.0]], [[0.0, 0.0]], [[0.0, 0.0]]]])
    target = torch.tensor([[[0, 1]]])
    pixel_weights = torch.tensor([[[1.0, 8.0]]])

    unweighted = _bev_cross_entropy(logits, target, class_weights=None)
    weighted = _bev_cross_entropy(logits, target, class_weights=None, pixel_weights=pixel_weights)

    assert weighted > unweighted


def test_visibility_mode_combines_current_and_history_masks():
    config = _dummy_config()
    config.bev_visibility_mode = "union"
    model = build_joint_model(config)
    info = {
        "current_visible": torch.tensor([[[[1.0, 0.0]]]]),
        "history_observed": torch.tensor([[[[0.0, 1.0]]]]),
    }

    weights = model._bev_visibility_weights(info, start=0, frames=1)

    assert weights is not None
    torch.testing.assert_close(weights, torch.tensor([[[[4.0, 4.0]]]]))


def test_spatial_probe_head_preserves_sequence_and_map_shape():
    head = SpatialProbeHead(input_dim=32, output_channels=3, spatial=16)
    output = head(torch.randn(2, 4, 32))
    assert output.shape == (2, 4, 3, 16, 16)


def test_occupied_dice_loss_is_finite_without_positive_targets():
    logits = torch.zeros(2, 3, 8, 8)
    target = torch.zeros(2, 8, 8, dtype=torch.long)
    loss = _occupied_dice_loss(logits, target)
    assert torch.isfinite(loss)


def test_visibility_weighted_occupied_dice_loss_stays_bounded():
    logits = torch.zeros(1, 3, 2, 2)
    target = torch.tensor([[[1, 0], [0, 1]]])
    pixel_weights = torch.tensor([[[4.0, 0.25], [4.0, 0.25]]])

    loss = _occupied_dice_loss(logits, target, pixel_weights=pixel_weights)

    assert torch.isfinite(loss)
    assert 0.0 <= float(loss) <= 1.0


def test_build_joint_model_selects_spatial_probe_backend():
    config = _dummy_config()
    config.bev_decoder = "spatial"
    model = build_joint_model(config)
    assert isinstance(model.bev_head, SpatialProbeHead)
    assert isinstance(model.current_head, SpatialProbeHead)
    assert isinstance(model.history_head, SpatialProbeHead)


def test_joint_losses_include_finite_closed_loop_terms():
    config = JointModelConfig(
        image_size=32,
        embed_dim=32,
        bev_size=16,
        history_size=3,
        num_preds=1,
        rollout_horizon=3,
        rollout_loss_weight=0.25,
    )
    model = build_joint_model(config)
    model.encoder = _FakeEncoder(config.embed_dim)
    info = {
        "pixels_left": torch.randn(2, 6, 3, 32, 32),
        "pixels_right": torch.randn(2, 6, 3, 32, 32),
        "action": torch.randn(2, 6, 10),
        "proprio": torch.randn(2, 6, 17),
        "geometry_bev": torch.randint(0, 3, (2, 6, 16, 16)),
        "current_visible": torch.randint(0, 2, (2, 6, 16, 16)).float(),
        "history_observed": torch.randint(0, 2, (2, 6, 16, 16)).float(),
    }
    losses = model.losses(model(info), SIGReg(knots=5, num_proj=32))
    assert torch.isfinite(losses["rollout_latent_loss"])
    assert torch.isfinite(losses["rollout_bev_loss"])


def _pose_batch(batch: int, frames: int, horizon: int) -> dict[str, torch.Tensor]:
    target = torch.zeros(batch, horizon, 4)
    target[..., 3] = 1.0
    return {
        "pixels_left": torch.randn(batch, frames, 3, 32, 32),
        "pixels_right": torch.randn(batch, frames, 3, 32, 32),
        "action": torch.randn(batch, frames, 10),
        "proprio": torch.randn(batch, frames, 17),
        "geometry_bev": torch.randint(0, 3, (batch, frames, 16, 16)),
        "current_visible": torch.randint(0, 2, (batch, frames, 16, 16)).float(),
        "history_observed": torch.randint(0, 2, (batch, frames, 16, 16)).float(),
        "future_relative_pose": target,
    }


def test_no_spatial_supervision_has_no_spatial_gradients_or_label_dependence():
    config = JointModelConfig(image_size=32, embed_dim=32, bev_size=16,
                              history_size=3, rollout_horizon=5, pose_head_enabled=True)
    config.spatial_supervision_enabled = False
    model = build_joint_model(config)
    model.encoder = _FakeEncoder(config.embed_dim)
    batch = _pose_batch(1, 8, 5)
    output = model(batch)
    sigreg = SIGReg(knots=5, num_proj=16)
    torch.manual_seed(1)
    losses = model.losses(output, sigreg)
    for key in ('bev_loss', 'current_loss', 'history_loss', 'rollout_bev_loss'):
        assert losses[key].item() == 0.0
    losses['loss'].backward()
    for head in (model.bev_head, model.current_head, model.history_head):
        assert all(p.grad is None for p in head.parameters())
    assert any(p.grad is not None and p.grad.abs().sum() > 0 for p in model.pose_head.parameters())
    for key in ('geometry_bev', 'current_visible', 'history_observed'):
        output.pop(key)
    torch.manual_seed(1)
    without_labels = model.losses(output, sigreg)
    assert torch.allclose(losses['loss'], without_labels['loss'])


def test_spatial_ablation_preserves_common_random_initialization():
    config = _dummy_config()
    torch.manual_seed(3072)
    with_spatial = build_joint_model(config)
    config.spatial_supervision_enabled = False
    torch.manual_seed(3072)
    without_spatial = build_joint_model(config)
    a, b = with_spatial.state_dict(), without_spatial.state_dict()
    assert a.keys() == b.keys()
    assert all(torch.equal(a[key], b[key]) for key in a)


def test_pose_enabled_model_decodes_normalized_yaw_and_backpropagates():
    """Catches a pose head that omits yaw normalization or receives no loss gradient."""

    config = JointModelConfig(
        image_size=32,
        embed_dim=32,
        bev_size=16,
        history_size=3,
        rollout_horizon=5,
        pose_head_enabled=True,
        pose_loss_weight=1.0,
        pose_horizons=(1, 3, 5),
    )
    model = build_joint_model(config)
    model.encoder = _FakeEncoder(config.embed_dim)

    output = model(_pose_batch(batch=2, frames=8, horizon=5))
    losses = model.losses(output, SIGReg(knots=5, num_proj=32))

    assert output["rollout_relative_pose"].shape == (2, 5, 4)
    assert torch.allclose(
        output["rollout_relative_pose"][..., 2:].norm(dim=-1),
        torch.ones(2, 5),
        atol=1e-5,
    )
    assert torch.isfinite(losses["rollout_pose_loss"])
    losses["loss"].backward()
    assert any(parameter.grad is not None for parameter in model.pose_head.parameters())


def test_relative_pose_loss_uses_only_configured_rollout_horizons():
    """Catches a loss that accidentally supervises the unrequested second/fourth rollout steps."""

    pose_loss = getattr(joint_jepa, "_relative_pose_loss", None)
    if pose_loss is None:
        pytest.fail("_relative_pose_loss is not available")
    target = torch.zeros(1, 5, 4)
    target[..., 3] = 1.0
    unselected_error = target.clone()
    unselected_error[:, 1, 0] = 10.0
    selected_error = target.clone()
    selected_error[:, 2, 0] = 1.0

    assert pose_loss(target, target, horizons=(1, 3, 5)) == pytest.approx(0.0, abs=1e-6)
    assert pose_loss(unselected_error, target, horizons=(1, 3, 5)) == pytest.approx(0.0, abs=1e-6)
    assert pose_loss(selected_error, target, horizons=(1, 3, 5)) > 0.0


def test_legacy_pose_disabled_config_loads_strictly_without_pose_parameters():
    """Catches default pose parameters that would make old checkpoint loading silently non-strict."""

    legacy = build_joint_model(_dummy_config())
    legacy_config = asdict(_dummy_config())
    legacy_config.pop("action_source", None)
    restored_config = JointModelConfig(**legacy_config)
    restored = build_joint_model(restored_config)

    restored.load_state_dict(legacy.state_dict(), strict=True)

    assert restored.pose_head is None
    assert restored_config.action_source == "applied_action"


def test_rollout_uses_first_candidate_action_for_first_prediction():
    model = _tiny_rollout_model()
    embed_dim = 16

    def encode(info):
        batch, frames = info["pixels_left"].shape[:2]
        return {"emb": torch.zeros(batch, frames, embed_dim)}

    def predict(self, emb, act_emb):
        return act_emb[..., :1].expand(*act_emb.shape[:-1], embed_dim)

    model.encode = encode
    model.action_encoder = torch.nn.Identity()
    model.predict = types.MethodType(predict, model)
    context = {
        "pixels_left": torch.zeros(1, 3, 3, 8, 8),
        "pixels_right": torch.zeros(1, 3, 3, 8, 8),
        "action": torch.zeros(1, 3, 10),
        "proprio": torch.zeros(1, 3, 17),
    }
    future_actions = torch.zeros(1, 1, 2, 10)
    future_actions[:, :, 0, 0] = 7.0

    predicted = model.rollout(context, future_actions, history_size=3)

    assert predicted.shape == (1, 1, 2, embed_dim)
    assert torch.all(predicted[0, 0, 0] == 7.0)


def test_rollout_conditions_each_prediction_on_its_candidate_action_in_order():
    """Catches delayed candidate actions and stale final context actions after step one."""

    torch.manual_seed(31)
    model = _tiny_rollout_model().eval()
    # AdaLN starts at zero and therefore ignores actions before learning. Use
    # nonzero weights to exercise the real action-conditioned predictor.
    for block in model.predictor.transformer.layers:
        torch.nn.init.normal_(block.adaLN_modulation[-1].weight, std=0.2)
    context = {
        "pixels_left": torch.randn(2, 3, 3, 8, 8),
        "pixels_right": torch.randn(2, 3, 3, 8, 8),
        "action": torch.randn(2, 3, 10),
        "proprio": torch.randn(2, 3, 17),
    }
    context["action"][:, -1] = 9.0  # Placeholder replaced by candidate a0.
    context_action_before = context["action"].clone()
    actions = torch.randn(2, 2, 3, 10)
    candidate_actions_before = actions.clone()

    with torch.no_grad():
        encoded = model.encode(dict(context))["emb"]
        emb = encoded.repeat_interleave(2, dim=0)
        past = context["action"].repeat_interleave(2, dim=0)
        a0, a1, a2 = actions.reshape(4, 3, 10).split(1, dim=1)
        # Explicitly pair each state with the action that leads to the next
        # frame: [c0,c1,a0], [c1,a0,a1], then [a0,a1,a2].
        p0 = model.predict(
            emb, model.action_encoder(torch.cat((past[:, :2], a0), dim=1))
        )[:, -1:]
        p1 = model.predict(
            torch.cat((emb[:, 1:], p0), dim=1),
            model.action_encoder(torch.cat((past[:, 1:2], a0, a1), dim=1)),
        )[:, -1:]
        p2 = model.predict(
            torch.cat((emb[:, 2:], p0, p1), dim=1),
            model.action_encoder(torch.cat((a0, a1, a2), dim=1)),
        )[:, -1:]
        expected = torch.cat((p0, p1, p2), dim=1).reshape(2, 2, 3, 16)

        predicted = model.rollout(context, actions, history_size=3)
        # Candidates sharing a0/a1 must have the same first two predictions,
        # while changing a2 must affect the third prediction.
        changed_actions = actions.clone()
        changed_actions[:, :, 2] += 2.0
        changed = model.rollout(context, changed_actions, history_size=3)

    torch.testing.assert_close(predicted[:, :, 0], expected[:, :, 0])
    torch.testing.assert_close(predicted[:, :, 1], expected[:, :, 1])
    torch.testing.assert_close(predicted[:, :, 2], expected[:, :, 2])
    torch.testing.assert_close(changed[:, :, :2], predicted[:, :, :2])
    assert not torch.allclose(changed[:, :, 2], predicted[:, :, 2])
    torch.testing.assert_close(context["action"], context_action_before)
    torch.testing.assert_close(actions, candidate_actions_before)


def _tiny_rollout_model() -> JointJEPA:
    config_embed = 16
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
        encoder=torch.nn.Sequential(
            torch.nn.Flatten(), torch.nn.Linear(6 * 8 * 8, config_embed)
        ),
        predictor=predictor,
        action_encoder=Embedder(10, emb_dim=config_embed),
        proprio_encoder=torch.nn.Linear(17, config_embed),
        bev_head=_ProbeHead(config_embed, 3, 16),
        current_head=_ProbeHead(config_embed, 1, 16),
        history_head=_ProbeHead(config_embed, 1, 16),
        projector=MLP(config_embed, 32, config_embed),
        pred_proj=MLP(config_embed, 32, config_embed),
    )
    model.history_size = 3
    return model
