"""Legacy-checkpoint world-model adapter for the PointGoal benchmark.

The checkpoints produced before Benchmark V1 use the narrower ``aux16``
action range.  This adapter keeps candidate search inside that range and
returns actions that are still valid under the current V2 benchmark contract.
It is intentionally labelled as a diagnostic adapter; it does not silently
turn a legacy checkpoint into a V2-trained policy.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np
import torch
import torch.nn.functional as F

from training.action_protocol import ActionProtocol, declared_action_protocol_id
from training.joint_jepa import JointModelConfig, build_joint_model
from training.planner import plan
from training.proprioception import checkpoint_velocity_frame, transform_proprio_velocities

from .policies import PolicyObservation, _PolicyBase


LEGACY_AUXILIARY_LIMIT_RAD = 0.2809658115
LEGACY_PROTOCOL = ActionProtocol(
    protocol_id="ball_action_v1_aux16",
    minimum=(-2.0, -LEGACY_AUXILIARY_LIMIT_RAD),
    maximum=(2.0, LEGACY_AUXILIARY_LIMIT_RAD),
    controls_per_block=5,
)
IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406])
IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225])


def _image_tensor(image: np.ndarray, image_size: int) -> torch.Tensor:
    value = np.asarray(image)
    if value.shape != (480, 640, 3) or value.dtype != np.uint8:
        raise ValueError("benchmark RGB observations must be uint8 arrays shaped (480, 640, 3)")
    tensor = torch.from_numpy(np.ascontiguousarray(value).copy()).permute(2, 0, 1).float() / 255.0
    tensor = F.interpolate(
        tensor.unsqueeze(0), size=(image_size, image_size), mode="bilinear", align_corners=False
    ).squeeze(0)
    return (tensor - IMAGENET_MEAN[:, None, None]) / IMAGENET_STD[:, None, None]


class WorldModelPolicy(_PolicyBase):
    """Closed-loop pose-CEM policy backed by a previously trained checkpoint."""

    policy_name = "world_model_legacy"

    def __init__(
        self,
        protocol: ActionProtocol,
        *,
        checkpoint: str | Path,
        device: str = "cuda",
        horizon: int = 5,
        samples: int = 64,
        elites: int = 8,
        iterations: int = 3,
        safety_weight: float = 0.0,
        allow_legacy_checkpoint: bool = False,
    ) -> None:
        super().__init__(protocol)
        if horizon < 1 or samples < 1 or elites < 1 or iterations < 1:
            raise ValueError("world-model planner settings must be positive")
        if elites > samples:
            raise ValueError("world-model elites cannot exceed samples")
        if not np.isfinite(float(safety_weight)) or float(safety_weight) < 0.0:
            raise ValueError("world-model safety_weight must be finite and non-negative")
        self.checkpoint_path = Path(checkpoint).expanduser().resolve()
        if not self.checkpoint_path.is_file():
            raise FileNotFoundError(f"world-model checkpoint does not exist: {self.checkpoint_path}")
        requested_device = str(device)
        self.device = torch.device(
            requested_device
            if not requested_device.startswith("cuda") or torch.cuda.is_available()
            else "cpu"
        )
        payload = torch.load(self.checkpoint_path, map_location=self.device, weights_only=False)
        if not isinstance(payload, Mapping) or "config" not in payload or "model" not in payload:
            raise ValueError("world-model checkpoint must contain config and model")
        self.checkpoint_protocol_id = declared_action_protocol_id(payload)
        if self.checkpoint_protocol_id != protocol.protocol_id and not allow_legacy_checkpoint:
            raise ValueError(
                f"checkpoint uses {self.checkpoint_protocol_id}; pass --allow-legacy-checkpoint "
                "to run the explicitly labelled diagnostic adapter"
            )
        if self.checkpoint_protocol_id == "ball_action_v2_aux30":
            self.model_protocol = protocol
        elif self.checkpoint_protocol_id == LEGACY_PROTOCOL.protocol_id:
            self.model_protocol = LEGACY_PROTOCOL
        else:
            raise ValueError(f"unsupported checkpoint action protocol: {self.checkpoint_protocol_id}")
        config_values = dict(payload["config"])
        config_values["proprio_velocity_frame"] = checkpoint_velocity_frame(payload)
        config = JointModelConfig(**config_values)
        if config.action_source not in ("applied_action", "commanded_action"):
            raise ValueError("unsupported checkpoint action_source")
        if payload.get("action_source", config.action_source) != config.action_source:
            raise ValueError("checkpoint action_source metadata disagrees with config")
        if not config.pose_head_enabled:
            raise ValueError("world-model benchmark adapter requires a pose-enabled checkpoint")
        self.config = config
        self.model = build_joint_model(config).to(self.device)
        self.model.load_state_dict(payload["model"], strict=True)
        self.model.eval()
        self.horizon = int(horizon)
        self.samples = int(samples)
        self.elites = int(elites)
        self.iterations = int(iterations)
        self.safety_weight = float(safety_weight)
        self._left_history: list[torch.Tensor] = []
        self._right_history: list[torch.Tensor] = []
        self.diagnostics: dict[str, object] = {}

    @property
    def online_input_permissions(self) -> frozenset[str]:
        permissions = {
            "stereo_rgb",
            "proprio_history",
            "action_history",
            "perfect_position_yaw",
            "pointgoal_base_xy",
        }
        if getattr(self.config, "proprio_velocity_frame", "world") == "base":
            permissions.add("quaternion_world_wxyz_history")
        return frozenset(permissions)

    @property
    def adapter_metadata(self) -> dict[str, object]:
        return {
            "checkpoint": str(self.checkpoint_path),
            "checkpoint_protocol_id": self.checkpoint_protocol_id,
            "action_source": self.config.action_source,
            "proprio_velocity_frame": self.config.proprio_velocity_frame,
            "planner_protocol_id": self.model_protocol.protocol_id,
            "adapter": "legacy_aux16_to_benchmark_v2",
            "legacy_auxiliary_limit_rad": LEGACY_AUXILIARY_LIMIT_RAD,
            "device": str(self.device),
            "horizon": self.horizon,
            "samples": self.samples,
            "elites": self.elites,
            "iterations": self.iterations,
            "safety_weight": self.safety_weight,
        }

    def reset(self, context: Any) -> None:
        self._reset_non_privileged(context)
        self._left_history = []
        self._right_history = []
        self.diagnostics = {}

    def _model_context(self, observation: PolicyObservation) -> dict[str, torch.Tensor]:
        left = _image_tensor(observation.stereo_left_rgb, self.config.image_size)
        right = _image_tensor(observation.stereo_right_rgb, self.config.image_size)
        self._left_history.append(left)
        self._right_history.append(right)
        self._left_history = self._left_history[-self.config.history_size :]
        self._right_history = self._right_history[-self.config.history_size :]
        if not self._left_history or not self._right_history:
            raise RuntimeError("world-model image history is empty")
        left_history = [self._left_history[0]] * (self.config.history_size - len(self._left_history)) + self._left_history
        right_history = [self._right_history[0]] * (self.config.history_size - len(self._right_history)) + self._right_history
        proprio = torch.from_numpy(transform_proprio_velocities(
            observation.proprio_history,
            observation.quaternion_world_wxyz_history,
            frame=getattr(self.config, "proprio_velocity_frame", "world"),
        ))
        actions = torch.from_numpy(np.array(observation.action_history, dtype=np.float32, copy=True))
        if proprio.shape != (self.config.history_size, self.config.proprio_dim):
            raise ValueError("benchmark proprio history does not match checkpoint config")
        if actions.shape != (self.config.history_size, self.config.action_dim):
            raise ValueError("benchmark action history does not match checkpoint config")
        if getattr(self.config, "action_source", "applied_action") == "commanded_action":
            # Runtime histories contain incoming requests. Dataset actions describe
            # the transition leaving each frame, so retain the latest completed block.
            actions = torch.cat((actions[1:], torch.zeros_like(actions[:1])), dim=0)
        # The planner replaces this row with its first candidate action.
        actions[-1] = 0.0
        return {
            "pixels_left": torch.stack(left_history).unsqueeze(0),
            "pixels_right": torch.stack(right_history).unsqueeze(0),
            "proprio": proprio.unsqueeze(0),
            "action": actions.unsqueeze(0),
        }

    def act(self, observation: PolicyObservation, pointgoal_base_xy: object) -> np.ndarray:
        pointgoal = self._validate_act_inputs(observation, pointgoal_base_xy)
        context = self._model_context(observation)
        goal = torch.from_numpy(pointgoal.astype(np.float32)).reshape(1, 2)
        with torch.no_grad():
            result = plan(
                self.model,
                context,
                action_protocol=self.model_protocol,
                horizon=self.horizon,
                samples=self.samples,
                elites=self.elites,
                iterations=self.iterations,
                safety_weight=self.safety_weight,
                history_size=self.config.history_size,
                device=self.device,
                planner_mode="pose_bev",
                goal_relative_xy=goal,
            )
        action = result.actions[0].detach().cpu().numpy().astype(np.float32)
        self.diagnostics = {
            "total_cost": float(result.cost),
            "goal_cost": float(result.goal_cost),
            "safety_cost": float(result.safety_cost),
            "fallback_used": bool(result.fallback_used),
        }
        return self.protocol.clip_block(action)
