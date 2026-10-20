"""Versioned action bounds shared by collection, training, and planning."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any, Mapping

import numpy as np
import torch


LEGACY_ACTION_PROTOCOL_ID = "ball_action_v1_aux16"


def declared_action_protocol_id(record: Mapping[str, Any]) -> str:
    """Return a declared protocol ID, treating unlabelled artifacts as legacy V1."""

    value = record.get("action_protocol_id")
    if value is None:
        return LEGACY_ACTION_PROTOCOL_ID
    if not isinstance(value, str) or not value.strip():
        raise ValueError("action_protocol_id must be a non-empty string")
    return value


def require_action_protocol(
    record: Mapping[str, Any], expected_protocol_id: str, *, artifact: str
) -> str:
    """Reject an artifact whose declared/legacy protocol differs from the evaluator."""

    actual = declared_action_protocol_id(record)
    if actual != expected_protocol_id:
        raise ValueError(
            f"{artifact} uses {actual}, but evaluation requires {expected_protocol_id}"
        )
    return actual


@dataclass(frozen=True)
class ActionProtocol:
    """Immutable action-block layout parsed from the collection contract."""

    protocol_id: str
    minimum: tuple[float, float]
    maximum: tuple[float, float]
    controls_per_block: int

    @classmethod
    def from_contract(cls, contract: Any) -> "ActionProtocol":
        return cls(
            protocol_id=contract.action_protocol_id,
            minimum=tuple(contract.action_minimum),
            maximum=tuple(contract.action_maximum),
            controls_per_block=contract.controls_per_block,
        )

    @property
    def action_block_dim(self) -> int:
        return self.controls_per_block * len(self.minimum)

    def component_bounds_numpy(self) -> np.ndarray:
        return np.stack((self.minimum, self.maximum), axis=1).astype(np.float32)

    def block_bounds_numpy(self) -> np.ndarray:
        return np.tile(self.component_bounds_numpy(), (self.controls_per_block, 1))

    def block_bounds_torch(self) -> torch.Tensor:
        return torch.from_numpy(self.block_bounds_numpy())

    def clip_block(self, action_block: Any) -> np.ndarray:
        values = np.asarray(action_block, dtype=np.float32)
        if values.shape != (self.action_block_dim,):
            raise ValueError(
                f"action block must have shape ({self.action_block_dim},), got {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("action block must contain only finite values")
        bounds = self.block_bounds_numpy()
        return np.clip(values, bounds[:, 0], bounds[:, 1]).astype(np.float32)

    def sample_block(self, generator: np.random.Generator) -> np.ndarray:
        if not isinstance(generator, np.random.Generator):
            raise TypeError("generator must be a numpy.random.Generator")
        bounds = self.block_bounds_numpy()
        return generator.uniform(bounds[:, 0], bounds[:, 1]).astype(np.float32)


def load_action_protocol(contract_path: str | Path) -> ActionProtocol:
    """Load one validated protocol from the shared collection contract."""

    scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    from collection_schema import load_contract

    return ActionProtocol.from_contract(load_contract(Path(contract_path)))
