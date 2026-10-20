"""Joint stereo+proprioception JEPA with BEV and visibility probe heads."""

from __future__ import annotations

from dataclasses import dataclass

import timm
import torch
from torch import nn
import torch.nn.functional as F
from einops import rearrange

from .lewm_modules import ARPredictor, Embedder, MLP, SIGReg


@dataclass
class JointModelConfig:
    image_size: int = 224
    embed_dim: int = 192
    action_dim: int = 10
    proprio_dim: int = 17
    bev_classes: int = 3
    bev_size: int = 80
    history_size: int = 5
    num_preds: int = 1
    rollout_horizon: int = 1
    rollout_loss_weight: float = 0.25
    encoder_name: str = "vit_tiny_patch16_224"
    predictor_depth: int = 6
    predictor_heads: int = 8
    predictor_mlp_dim: int = 768
    predictor_hidden_dim: int = 192
    predictor_dim_head: int = 64
    predictor_dropout: float = 0.0
    bev_class_weights: tuple[float, ...] = (1.0, 8.0, 1.0)
    bev_decoder: str = "mlp"
    bev_dice_weight: float = 0.5
    bev_visibility_mode: str = "none"
    bev_visible_weight: float = 4.0
    bev_unobserved_weight: float = 0.25
    spatial_supervision_enabled: bool = True
    pose_head_enabled: bool = False
    pose_loss_weight: float = 1.0
    pose_horizons: tuple[int, ...] = (1, 3, 5)


def build_default_encoder(config: JointModelConfig) -> nn.Module:
    """Shared ViT accepting 6-channel stereo input."""

    return timm.create_model(
        config.encoder_name,
        pretrained=False,
        in_chans=6,
        num_classes=0,
        img_size=config.image_size,
    )


class _ProbeHead(nn.Module):
    def __init__(self, input_dim: int, output_channels: int, spatial: int):
        super().__init__()
        self.spatial = spatial
        self.output_channels = output_channels
        self.net = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.GELU(),
            nn.Linear(512, output_channels * spatial * spatial),
        )

    def forward(self, emb: torch.Tensor) -> torch.Tensor:
        leading = emb.shape[:-1]
        logits = self.net(emb.reshape(-1, emb.size(-1)))
        return logits.reshape(
            *leading,
            self.output_channels,
            self.spatial,
            self.spatial,
        )


class SpatialProbeHead(nn.Module):
    """Latent-conditioned 2D decoder with shared local convolutions."""

    def __init__(self, input_dim: int, output_channels: int, spatial: int):
        super().__init__()
        if input_dim < 1 or output_channels < 1 or spatial < 1:
            raise ValueError("input_dim, output_channels, and spatial must be positive")
        self.spatial = int(spatial)
        self.output_channels = int(output_channels)
        self.seed_size = max(1, int(round(self.spatial / 8)))
        self.seed = nn.Linear(input_dim, 128 * self.seed_size * self.seed_size)
        self.initial = nn.Conv2d(128, 128, kernel_size=3, padding=1)
        self.up1 = nn.Conv2d(128, 96, kernel_size=3, padding=1)
        self.up2 = nn.Conv2d(96, 64, kernel_size=3, padding=1)
        self.output = nn.Conv2d(64, self.output_channels, kernel_size=3, padding=1)

    def forward(self, emb: torch.Tensor) -> torch.Tensor:
        leading = emb.shape[:-1]
        x = self.seed(emb.reshape(-1, emb.size(-1)))
        x = x.reshape(-1, 128, self.seed_size, self.seed_size)
        x = F.gelu(self.initial(x))
        for layer in (self.up1, self.up2, self.output):
            x = F.interpolate(x, scale_factor=2.0, mode="bilinear", align_corners=False)
            x = layer(x)
            if layer is not self.output:
                x = F.gelu(x)
        if x.shape[-2:] != (self.spatial, self.spatial):
            x = F.interpolate(x, size=(self.spatial, self.spatial), mode="bilinear", align_corners=False)
        return x.reshape(*leading, self.output_channels, self.spatial, self.spatial)


def _bev_cross_entropy(
    logits: torch.Tensor,
    target: torch.Tensor,
    class_weights: tuple[float, ...] | None,
    pixel_weights: torch.Tensor | None = None,
) -> torch.Tensor:
    """Cross-entropy for BEV labels with optional class balancing."""

    weight = None if class_weights is None else logits.new_tensor(class_weights)
    if pixel_weights is None:
        return nn.functional.cross_entropy(logits, target, weight=weight)
    if pixel_weights.shape != target.shape:
        raise ValueError("pixel_weights must match BEV target shape")
    if (pixel_weights < 0.0).any() or not torch.isfinite(pixel_weights).all():
        raise ValueError("pixel_weights must be finite and non-negative")
    losses = nn.functional.cross_entropy(logits, target, weight=weight, reduction="none")
    if weight is None:
        normalization = pixel_weights.sum()
    else:
        normalization = (pixel_weights * weight[target]).sum()
    return (losses * pixel_weights).sum() / normalization.clamp_min(torch.finfo(losses.dtype).eps)


def _occupied_dice_loss(
    logits: torch.Tensor,
    target: torch.Tensor,
    smooth: float = 1.0,
    pixel_weights: torch.Tensor | None = None,
) -> torch.Tensor:
    """Soft Dice loss for the occupied BEV class, safe for empty targets."""

    occupied = (target == 1).to(dtype=logits.dtype)
    probability = logits.softmax(dim=1)[:, 1]
    if pixel_weights is not None:
        if pixel_weights.shape != target.shape:
            raise ValueError("pixel_weights must match BEV target shape")
        if (pixel_weights < 0.0).any() or not torch.isfinite(pixel_weights).all():
            raise ValueError("pixel_weights must be finite and non-negative")
        intersection = (probability * occupied * pixel_weights).sum(dim=(-2, -1))
        denominator = (probability * pixel_weights).sum(dim=(-2, -1)) + (
            occupied * pixel_weights
        ).sum(dim=(-2, -1))
    else:
        intersection = (probability * occupied).sum(dim=(-2, -1))
        denominator = probability.sum(dim=(-2, -1)) + occupied.sum(dim=(-2, -1))
    dice = (2.0 * intersection + smooth) / (denominator + smooth)
    return 1.0 - dice.mean()


def _relative_pose_loss(
    predicted: torch.Tensor,
    target: torch.Tensor,
    horizons: tuple[int, ...],
) -> torch.Tensor:
    """Huber XY plus unit-circle yaw loss at selected one-indexed horizons."""

    if predicted.ndim != 3 or target.ndim != 3 or predicted.shape != target.shape or predicted.size(-1) != 4:
        raise ValueError("predicted and target pose tensors must have matching shape (B, T, 4)")
    if not horizons or any(int(horizon) < 1 or int(horizon) > predicted.size(1) for horizon in horizons):
        raise ValueError("pose horizons must be non-empty valid one-indexed rollout positions")
    indices = torch.as_tensor(
        [int(horizon) - 1 for horizon in horizons],
        dtype=torch.long,
        device=predicted.device,
    )
    predicted_selected = predicted.index_select(1, indices)
    target_selected = target.index_select(1, indices)
    xy_loss = F.huber_loss(predicted_selected[..., :2], target_selected[..., :2])
    yaw_loss = F.mse_loss(predicted_selected[..., 2:4], target_selected[..., 2:4])
    return xy_loss + yaw_loss


class JointJEPA(nn.Module):
    """LeWM-style JEPA extended with stereo input, proprio, and BEV probes."""

    def __init__(
        self,
        encoder: nn.Module,
        predictor: nn.Module,
        action_encoder: nn.Module,
        proprio_encoder: nn.Module,
        bev_head: nn.Module,
        current_head: nn.Module,
        history_head: nn.Module,
        projector: nn.Module | None = None,
        pred_proj: nn.Module | None = None,
        bev_class_weights: tuple[float, ...] | None = None,
        bev_dice_weight: float = 0.0,
        bev_visibility_mode: str = "none",
        bev_visible_weight: float = 4.0,
        bev_unobserved_weight: float = 0.25,
        rollout_horizon: int = 1,
        rollout_loss_weight: float = 0.0,
        spatial_supervision_enabled: bool = True,
        pose_head: nn.Module | None = None,
        pose_loss_weight: float = 1.0,
        pose_horizons: tuple[int, ...] = (1, 3, 5),
    ) -> None:
        super().__init__()
        self.encoder = encoder
        self.predictor = predictor
        self.action_encoder = action_encoder
        self.proprio_encoder = proprio_encoder
        self.bev_head = bev_head
        self.current_head = current_head
        self.history_head = history_head
        self.spatial_supervision_enabled = bool(spatial_supervision_enabled)
        if not self.spatial_supervision_enabled:
            for head in (self.bev_head, self.current_head, self.history_head):
                head.requires_grad_(False)
        self.projector = projector or nn.Identity()
        self.pred_proj = pred_proj or nn.Identity()
        self.bev_class_weights = bev_class_weights
        if bev_dice_weight < 0.0:
            raise ValueError("bev_dice_weight must be non-negative")
        self.bev_dice_weight = float(bev_dice_weight)
        if bev_visibility_mode not in {"none", "current", "history", "union"}:
            raise ValueError("bev_visibility_mode must be one of none, current, history, union")
        for name, value in (
            ("bev_visible_weight", bev_visible_weight),
            ("bev_unobserved_weight", bev_unobserved_weight),
        ):
            if not torch.isfinite(torch.tensor(float(value))) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.bev_visibility_mode = str(bev_visibility_mode)
        self.bev_visible_weight = float(bev_visible_weight)
        self.bev_unobserved_weight = float(bev_unobserved_weight)
        if rollout_horizon < 1:
            raise ValueError("rollout_horizon must be positive")
        if rollout_loss_weight < 0.0:
            raise ValueError("rollout_loss_weight must be non-negative")
        self.rollout_horizon = int(rollout_horizon)
        self.rollout_loss_weight = float(rollout_loss_weight)
        if pose_loss_weight < 0.0:
            raise ValueError("pose_loss_weight must be non-negative")
        self.pose_head = pose_head
        self.pose_loss_weight = float(pose_loss_weight)
        self.pose_horizons = tuple(int(horizon) for horizon in pose_horizons)
        if self.pose_head is not None and (
            not self.pose_horizons
            or any(horizon < 1 or horizon > self.rollout_horizon for horizon in self.pose_horizons)
        ):
            raise ValueError("enabled pose head requires valid pose horizons within rollout_horizon")

    def _bev_visibility_weights(
        self,
        info: dict[str, torch.Tensor],
        *,
        start: int,
        frames: int,
    ) -> torch.Tensor | None:
        """Return per-cell BEV supervision weights for an observation mask."""

        if self.bev_visibility_mode == "none":
            return None
        if start < 0 or frames < 1:
            raise ValueError("BEV visibility slice must have non-negative start and positive frames")
        required = {
            "current": ("current_visible",),
            "history": ("history_observed",),
            "union": ("current_visible", "history_observed"),
        }[self.bev_visibility_mode]
        masks: list[torch.Tensor] = []
        for name in required:
            if name not in info:
                raise ValueError(f"visibility-aware BEV loss requires {name}")
            mask = info[name]
            if mask.ndim != 4 or start + frames > mask.size(1):
                raise ValueError(f"{name} must have shape (B,T,H,W) covering the requested BEV frames")
            masks.append(mask[:, start : start + frames])
        visible = masks[0] if len(masks) == 1 else torch.maximum(masks[0], masks[1])
        return torch.where(
            visible > 0,
            visible.new_tensor(self.bev_visible_weight),
            visible.new_tensor(self.bev_unobserved_weight),
        )

    def encode(self, info: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        left = info["pixels_left"].float()
        right = info["pixels_right"].float()
        batch = left.size(0)
        pixels = torch.cat([left, right], dim=2)  # (B,T,6,H,W)
        pixels = rearrange(pixels, "b t ... -> (b t) ...")
        features = self.encoder(pixels)
        if isinstance(features, tuple):
            features = features[0]
        emb = self.projector(features)
        emb = rearrange(emb, "(b t) d -> b t d", b=batch)
        proprio_emb = self.proprio_encoder(info["proprio"].float())
        info["emb"] = emb + proprio_emb
        info["act_emb"] = self.action_encoder(info["action"])
        return info

    def predict(self, emb: torch.Tensor, act_emb: torch.Tensor) -> torch.Tensor:
        preds = self.predictor(emb, act_emb)
        preds = self.pred_proj(rearrange(preds, "b t d -> (b t) d"))
        return rearrange(preds, "(b t) d -> b t d", b=emb.size(0))

    def decode_relative_pose(self, future_embeddings: torch.Tensor) -> torch.Tensor:
        """Decode relative XY and normalized sin/cos yaw from future latent embeddings."""

        if self.pose_head is None:
            raise RuntimeError("pose head is disabled")
        raw = self.pose_head(future_embeddings)
        if raw.shape[:-1] != future_embeddings.shape[:-1] or raw.size(-1) != 4:
            raise RuntimeError("pose head must preserve leading dimensions and output four values")
        yaw_unit = F.normalize(raw[..., 2:4], dim=-1, eps=1e-6)
        return torch.cat((raw[..., :2], yaw_unit), dim=-1)

    def forward(self, info: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        info = self.encode(info)
        emb = info["emb"]
        act_emb = info["act_emb"]
        context = emb[:, : self.predictor.pos_embedding.size(1)]
        context_act = act_emb[:, : context.size(1)]
        info["pred_emb"] = self.predict(context, context_act)
        if self.spatial_supervision_enabled:
            info["bev_logits"] = self.bev_head(emb)
            info["current_logits"] = self.current_head(emb)
            info["history_logits"] = self.history_head(emb)
        return info

    def losses(
        self,
        info: dict[str, torch.Tensor],
        sigreg: SIGReg,
        sigreg_weight: float = 0.09,
    ) -> dict[str, torch.Tensor]:
        emb = info["emb"]
        target = emb[:, self.num_preds : self.num_preds + self.predictor.pos_embedding.size(1)]
        predicted = info["pred_emb"]
        if predicted.size(1) > target.size(1):
            predicted = predicted[:, -target.size(1) :]
        elif target.size(1) > predicted.size(1):
            target = target[:, : predicted.size(1)]
        pred_loss = (predicted - target).pow(2).mean()
        reg_loss = sigreg(emb.transpose(0, 1))
        bev_loss = current_loss = history_loss = emb.new_zeros(())
        if self.spatial_supervision_enabled:
            batch, frames, classes, height, width = info["bev_logits"].shape
            bev_pixel_weights = self._bev_visibility_weights(info, start=0, frames=frames)
            bev_loss = _bev_cross_entropy(
                info["bev_logits"].reshape(batch * frames, classes, height, width),
                info["geometry_bev"].reshape(batch * frames, height, width),
                self.bev_class_weights,
                None if bev_pixel_weights is None else bev_pixel_weights.reshape(batch * frames, height, width),
            )
            if self.bev_dice_weight > 0.0:
                bev_loss = bev_loss + self.bev_dice_weight * _occupied_dice_loss(
                    info["bev_logits"].reshape(batch * frames, classes, height, width),
                    info["geometry_bev"].reshape(batch * frames, height, width),
                    pixel_weights=None
                    if bev_pixel_weights is None
                    else bev_pixel_weights.reshape(batch * frames, height, width),
                )
            current_loss = nn.functional.binary_cross_entropy_with_logits(
                info["current_logits"].reshape(batch * frames, height, width),
                info["current_visible"].reshape(batch * frames, height, width),
            )
            history_loss = nn.functional.binary_cross_entropy_with_logits(
                info["history_logits"].reshape(batch * frames, height, width),
                info["history_observed"].reshape(batch * frames, height, width),
            )
        rollout_latent_loss = emb.new_zeros(())
        rollout_bev_loss = emb.new_zeros(())
        rollout_pose_loss = emb.new_zeros(())
        needs_rollout = self.rollout_horizon > 1 and (
            self.rollout_loss_weight > 0.0 or self.pose_head is not None
        )
        if needs_rollout:
            history = self.predictor.pos_embedding.size(1)
            available = min(
                emb.size(1) - history,
                info["action"].size(1) - history + 1,
                self.rollout_horizon,
            )
            if available > 0:
                candidate_actions = info["action"][:, history - 1 : history - 1 + available]
                predicted_future = self._rollout_embeddings(
                    emb[:, :history],
                    info["action"][:, :history],
                    candidate_actions.unsqueeze(1),
                    history,
                )[:, 0]
                target_future = emb[:, history : history + available]
                rollout_latent_loss = (predicted_future - target_future).pow(2).mean()
                if self.spatial_supervision_enabled:
                    rollout_bev_logits = self.bev_head(predicted_future)
                    rollout_batch, rollout_frames, rollout_classes, rollout_height, rollout_width = (
                        rollout_bev_logits.shape
                    )
                    rollout_target = info["geometry_bev"][:, history : history + available]
                    rollout_pixel_weights = self._bev_visibility_weights(
                        info,
                        start=history,
                        frames=available,
                    )
                    rollout_bev_loss = _bev_cross_entropy(
                        rollout_bev_logits.reshape(
                            rollout_batch * rollout_frames,
                            rollout_classes,
                            rollout_height,
                            rollout_width,
                        ),
                        rollout_target.reshape(rollout_batch * rollout_frames, rollout_height, rollout_width),
                        self.bev_class_weights,
                        None
                        if rollout_pixel_weights is None
                        else rollout_pixel_weights.reshape(rollout_batch * rollout_frames, rollout_height, rollout_width),
                    )
                    if self.bev_dice_weight > 0.0:
                        rollout_bev_loss = rollout_bev_loss + self.bev_dice_weight * _occupied_dice_loss(
                            rollout_bev_logits.reshape(
                                rollout_batch * rollout_frames,
                                rollout_classes,
                                rollout_height,
                                rollout_width,
                            ),
                                rollout_target.reshape(
                                    rollout_batch * rollout_frames,
                                    rollout_height,
                                    rollout_width,
                                ),
                                pixel_weights=None
                                if rollout_pixel_weights is None
                                else rollout_pixel_weights.reshape(
                                    rollout_batch * rollout_frames,
                                    rollout_height,
                                    rollout_width,
                                ),
                            )
                if self.pose_head is not None:
                    if "future_relative_pose" not in info:
                        raise ValueError("pose-enabled model requires future_relative_pose labels")
                    rollout_relative_pose = self.decode_relative_pose(predicted_future)
                    target_relative_pose = info["future_relative_pose"][:, :available].to(rollout_relative_pose)
                    if target_relative_pose.shape != rollout_relative_pose.shape:
                        raise ValueError(
                            "future_relative_pose must have shape (B, available_rollout_horizon, 4)"
                        )
                    info["rollout_relative_pose"] = rollout_relative_pose
                    rollout_pose_loss = _relative_pose_loss(
                        rollout_relative_pose,
                        target_relative_pose,
                        self.pose_horizons,
                    )
        total = (
            pred_loss
            + sigreg_weight * reg_loss
            + bev_loss
            + current_loss
            + history_loss
            + self.rollout_loss_weight * (rollout_latent_loss + rollout_bev_loss)
            + self.pose_loss_weight * rollout_pose_loss
        )
        return {
            "loss": total,
            "pred_loss": pred_loss,
            "sigreg_loss": reg_loss,
            "bev_loss": bev_loss,
            "current_loss": current_loss,
            "history_loss": history_loss,
            "rollout_latent_loss": rollout_latent_loss,
            "rollout_bev_loss": rollout_bev_loss,
            "rollout_pose_loss": rollout_pose_loss,
        }

    def _rollout_embeddings(
        self,
        emb: torch.Tensor,
        context_action: torch.Tensor,
        action_sequence: torch.Tensor,
        history_size: int | None = None,
    ) -> torch.Tensor:
        """Roll out already encoded context embeddings for candidate actions."""

        history = history_size or getattr(self, "history_size", self.predictor.pos_embedding.size(1))
        batch, samples, horizon, action_dim = action_sequence.shape
        emb = emb.unsqueeze(1).expand(batch, samples, -1, -1)
        act = context_action.float().unsqueeze(1).expand(batch, samples, -1, -1)
        emb = rearrange(emb, "b s t d -> (b s) t d")
        act = rearrange(act, "b s t a -> (b s) t a")

        for step in range(horizon):
            next_act = action_sequence[:, :, step, :].reshape(batch * samples, action_dim)
            if step == 0:
                pred_act = act[:, -history:].clone()
                pred_act[:, -1] = next_act
            else:
                pred_act = act[:, -history:]
            act_emb = self.action_encoder(pred_act)
            pred = self.predict(emb[:, -history:], act_emb)[:, -1:]
            emb = torch.cat([emb, pred], dim=1)
            act = torch.cat([act, next_act.unsqueeze(1)], dim=1)

        future = emb[:, history:]
        return rearrange(future, "(b s) t d -> b s t d", b=batch, s=samples)

    def rollout(
        self,
        info: dict[str, torch.Tensor],
        action_sequence: torch.Tensor,
        history_size: int | None = None,
    ) -> torch.Tensor:
        """Autoregressively roll out future latent embeddings.

        ``info`` carries the context frames: ``pixels_left``, ``pixels_right``,
        ``proprio``, and the context ``action`` blocks, each with leading batch
        and time dimensions.  ``action_sequence`` is ``(B, S, T, A)`` candidate
        action blocks; its first block replaces the final context action and
        therefore controls the first predicted frame.  Returns predicted
        embeddings ``(B, S, T, D)``.
        """

        history = history_size or getattr(self, "history_size", self.predictor.pos_embedding.size(1))
        batch, samples, horizon, action_dim = action_sequence.shape
        encoded = self.encode(
            {key: value for key, value in info.items() if torch.is_tensor(value)}
        )
        return self._rollout_embeddings(
            encoded["emb"], info["action"], action_sequence, history
        )

    @property
    def num_preds(self) -> int:
        return getattr(self, "_num_preds", 1)

    @num_preds.setter
    def num_preds(self, value: int) -> None:
        self._num_preds = value


def build_joint_model(config: JointModelConfig) -> JointJEPA:
    encoder = build_default_encoder(config)
    predictor = ARPredictor(
        num_frames=config.history_size,
        depth=config.predictor_depth,
        heads=config.predictor_heads,
        mlp_dim=config.predictor_mlp_dim,
        input_dim=config.embed_dim,
        hidden_dim=config.predictor_hidden_dim,
        output_dim=config.embed_dim,
        dim_head=config.predictor_dim_head,
        dropout=config.predictor_dropout,
    )
    action_encoder = Embedder(input_dim=config.action_dim, emb_dim=config.embed_dim)
    proprio_encoder = nn.Sequential(
        nn.Linear(config.proprio_dim, config.embed_dim * 4),
        nn.SiLU(),
        nn.Linear(config.embed_dim * 4, config.embed_dim),
    )
    spatial = config.bev_size
    if config.bev_decoder == "mlp":
        probe_head = _ProbeHead
    elif config.bev_decoder == "spatial":
        probe_head = SpatialProbeHead
    else:
        raise ValueError(f"unsupported BEV decoder: {config.bev_decoder}")
    bev_head = probe_head(config.embed_dim, config.bev_classes, spatial)
    current_head = probe_head(config.embed_dim, 1, spatial)
    history_head = probe_head(config.embed_dim, 1, spatial)
    projector = MLP(config.embed_dim, config.embed_dim * 4, config.embed_dim)
    pred_proj = MLP(config.embed_dim, config.embed_dim * 4, config.embed_dim)
    pose_head = None
    if config.pose_head_enabled:
        pose_head = nn.Sequential(
            nn.Linear(config.embed_dim, config.embed_dim * 2),
            nn.SiLU(),
            nn.Linear(config.embed_dim * 2, 4),
        )
    model = JointJEPA(
        encoder=encoder,
        predictor=predictor,
        action_encoder=action_encoder,
        proprio_encoder=proprio_encoder,
        bev_head=bev_head,
        current_head=current_head,
        history_head=history_head,
        projector=projector,
        pred_proj=pred_proj,
        bev_class_weights=tuple(config.bev_class_weights),
        bev_dice_weight=config.bev_dice_weight,
        bev_visibility_mode=config.bev_visibility_mode,
        bev_visible_weight=config.bev_visible_weight,
        bev_unobserved_weight=config.bev_unobserved_weight,
        rollout_horizon=config.rollout_horizon,
        rollout_loss_weight=config.rollout_loss_weight,
        spatial_supervision_enabled=config.spatial_supervision_enabled,
        pose_head=pose_head,
        pose_loss_weight=config.pose_loss_weight,
        pose_horizons=tuple(int(horizon) for horizon in config.pose_horizons),
    )
    model.num_preds = config.num_preds
    model.history_size = config.history_size
    return model
