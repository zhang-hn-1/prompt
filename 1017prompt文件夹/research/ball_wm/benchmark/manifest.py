"""Immutable, self-verifying records for Ball PointGoal Benchmark V1."""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Any, Literal, Mapping


BENCHMARK_SCHEMA_VERSION = "ball_pointgoal_benchmark_v1"
EPISODE_SCHEMA_VERSION = "benchmark_episode_v1"
BENCHMARK_VERSION = "v1"
ACTION_PROTOCOL_ID = "ball_action_v2_aux30"
LOCALIZATION_SOURCE = "perfect_localization"
SCENE_ROOT_PREFIX = "${SCENE_ROOT}/"

_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_EPISODE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_SPLITS = frozenset(("golden", "dev", "test"))
_DIFFICULTIES = frozenset(("E0", "E1", "E2"))
_REVIEW_STATUSES = frozenset(("approved", "rejected", "needs_adjustment"))


def _require_nonempty_text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")
    return value


def _require_sha256(value: object, field: str) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise ValueError(f"{field} must be a lowercase 64-character SHA256")
    return value


def _finite_float(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be finite numeric data")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must contain only finite numeric data")
    return result


def _finite_vector(value: object, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{field} must contain {length} finite values")
    return tuple(_finite_float(item, field) for item in value)


def _freeze_json_value(value: object, field: str) -> object:
    """Validate JSON-compatible evidence and recursively make it immutable."""

    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{field} must contain only finite numeric data")
        return value
    if isinstance(value, Mapping):
        frozen: dict[str, object] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{field} keys must be strings")
            frozen[key] = _freeze_json_value(item, field)
        return MappingProxyType(frozen)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_json_value(item, field) for item in value)
    raise ValueError(f"{field} must contain JSON-compatible values")


def _plain_json_value(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): _plain_json_value(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_plain_json_value(item) for item in value]
    return value


def _canonical_json_bytes(payload: Mapping[str, object]) -> bytes:
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class HumanReview:
    status: Literal["approved", "rejected", "needs_adjustment"]
    reviewer: str
    notes: str

    def __post_init__(self) -> None:
        if self.status not in _REVIEW_STATUSES:
            raise ValueError(f"unsupported human review status: {self.status!r}")
        _require_nonempty_text(self.reviewer, "human_review.reviewer")
        if not isinstance(self.notes, str):
            raise ValueError("human_review.notes must be a string")

    def to_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "reviewer": self.reviewer,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "HumanReview":
        return cls(
            status=payload["status"],  # type: ignore[arg-type]
            reviewer=payload["reviewer"],  # type: ignore[arg-type]
            notes=payload["notes"],  # type: ignore[arg-type]
        )


@dataclass(frozen=True)
class BenchmarkEpisode:
    episode_id: str
    split: Literal["golden", "dev", "test"]
    difficulty: Literal["E0", "E1", "E2"]
    scene_uri: str
    scene_sha256: str
    occupancy_sha256: str
    start_position_world_m: tuple[float, float, float]
    start_quaternion_world_wxyz: tuple[float, float, float, float]
    goal_position_world_m: tuple[float, float, float]
    euclidean_distance_m: float
    oracle_path_length_m: float
    seed: int
    human_review: HumanReview
    classification_evidence: Mapping[str, object]

    def __post_init__(self) -> None:
        if not isinstance(self.episode_id, str) or _EPISODE_ID_RE.fullmatch(self.episode_id) is None:
            raise ValueError("episode_id must be a portable non-empty identifier")
        if self.split not in _SPLITS:
            raise ValueError(f"unsupported benchmark split: {self.split!r}")
        if self.difficulty not in _DIFFICULTIES:
            raise ValueError(f"unsupported benchmark difficulty: {self.difficulty!r}")
        self._validated_scene_relative_path()
        _require_sha256(self.scene_sha256, "scene_sha256")
        _require_sha256(self.occupancy_sha256, "occupancy_sha256")

        start = _finite_vector(self.start_position_world_m, 3, "start_position_world_m")
        quaternion = _finite_vector(
            self.start_quaternion_world_wxyz,
            4,
            "start_quaternion_world_wxyz",
        )
        goal = _finite_vector(self.goal_position_world_m, 3, "goal_position_world_m")
        quaternion_norm = math.sqrt(sum(component * component for component in quaternion))
        if not math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
            raise ValueError("start_quaternion_world_wxyz must be a finite unit quaternion")

        euclidean = _finite_float(self.euclidean_distance_m, "euclidean_distance_m")
        oracle = _finite_float(self.oracle_path_length_m, "oracle_path_length_m")
        if euclidean <= 0.0 or oracle <= 0.0:
            raise ValueError("episode path distances must be positive finite values")
        if oracle + 1e-9 < euclidean:
            raise ValueError("oracle_path_length_m cannot be shorter than euclidean_distance_m")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or self.seed < 0:
            raise ValueError("seed must be a non-negative integer")
        if not isinstance(self.human_review, HumanReview):
            raise ValueError("human_review must be a HumanReview record")

        evidence = _freeze_json_value(
            self.classification_evidence,
            "classification_evidence",
        )
        if not isinstance(evidence, Mapping):
            raise ValueError("classification_evidence must be a mapping")

        object.__setattr__(self, "start_position_world_m", start)
        object.__setattr__(self, "start_quaternion_world_wxyz", quaternion)
        object.__setattr__(self, "goal_position_world_m", goal)
        object.__setattr__(self, "euclidean_distance_m", euclidean)
        object.__setattr__(self, "oracle_path_length_m", oracle)
        object.__setattr__(self, "classification_evidence", evidence)

    def _validated_scene_relative_path(self) -> PurePosixPath:
        if not isinstance(self.scene_uri, str):
            raise ValueError("scene_uri must use ${SCENE_ROOT}")
        if self.scene_uri.startswith("/"):
            raise ValueError("scene_uri cannot be an absolute path")
        if not self.scene_uri.startswith(SCENE_ROOT_PREFIX):
            raise ValueError("scene_uri must start with ${SCENE_ROOT}/")
        relative_text = self.scene_uri[len(SCENE_ROOT_PREFIX) :]
        relative = PurePosixPath(relative_text)
        if not relative_text or relative.is_absolute() or ".." in relative.parts:
            raise ValueError("scene_uri escapes SCENE_ROOT")
        if "\\" in relative_text:
            raise ValueError("scene_uri must use portable SCENE_ROOT paths")
        return relative

    def resolve_scene(self, scene_root: str | Path) -> Path:
        root = Path(scene_root).expanduser().resolve()
        relative = self._validated_scene_relative_path()
        resolved = root.joinpath(*relative.parts).resolve()
        try:
            resolved.relative_to(root)
        except ValueError as error:
            raise ValueError("scene_uri escapes SCENE_ROOT") from error
        return resolved

    def verify_scene(self, scene_root: str | Path) -> Path:
        scene_path = self.resolve_scene(scene_root)
        if not scene_path.is_file():
            raise FileNotFoundError(f"benchmark scene does not exist: {scene_path}")
        actual = _sha256_file(scene_path)
        if actual != self.scene_sha256:
            raise ValueError(
                f"scene SHA256 mismatch for {self.episode_id}: "
                f"expected {self.scene_sha256}, got {actual}"
            )
        return scene_path

    def verify_occupancy(self, occupancy_path: str | Path) -> Path:
        path = Path(occupancy_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"benchmark occupancy map does not exist: {path}")
        actual = _sha256_file(path)
        if actual != self.occupancy_sha256:
            raise ValueError(
                f"occupancy SHA256 mismatch for {self.episode_id}: "
                f"expected {self.occupancy_sha256}, got {actual}"
            )
        return path

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": EPISODE_SCHEMA_VERSION,
            "episode_id": self.episode_id,
            "split": self.split,
            "difficulty": self.difficulty,
            "scene_uri": self.scene_uri,
            "scene_sha256": self.scene_sha256,
            "occupancy_sha256": self.occupancy_sha256,
            "start_position_world_m": list(self.start_position_world_m),
            "start_quaternion_world_wxyz": list(self.start_quaternion_world_wxyz),
            "goal_position_world_m": list(self.goal_position_world_m),
            "euclidean_distance_m": self.euclidean_distance_m,
            "oracle_path_length_m": self.oracle_path_length_m,
            "seed": self.seed,
            "human_review": self.human_review.to_dict(),
            "classification_evidence": _plain_json_value(self.classification_evidence),
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "BenchmarkEpisode":
        schema_version = payload.get("schema_version")
        if schema_version != EPISODE_SCHEMA_VERSION:
            raise ValueError(f"unsupported benchmark episode schema: {schema_version!r}")
        review = payload.get("human_review")
        if not isinstance(review, Mapping):
            raise ValueError("human_review must be an object")
        evidence = payload.get("classification_evidence")
        if not isinstance(evidence, Mapping):
            raise ValueError("classification_evidence must be an object")
        return cls(
            episode_id=payload["episode_id"],  # type: ignore[arg-type]
            split=payload["split"],  # type: ignore[arg-type]
            difficulty=payload["difficulty"],  # type: ignore[arg-type]
            scene_uri=payload["scene_uri"],  # type: ignore[arg-type]
            scene_sha256=payload["scene_sha256"],  # type: ignore[arg-type]
            occupancy_sha256=payload["occupancy_sha256"],  # type: ignore[arg-type]
            start_position_world_m=payload["start_position_world_m"],  # type: ignore[arg-type]
            start_quaternion_world_wxyz=payload["start_quaternion_world_wxyz"],  # type: ignore[arg-type]
            goal_position_world_m=payload["goal_position_world_m"],  # type: ignore[arg-type]
            euclidean_distance_m=payload["euclidean_distance_m"],  # type: ignore[arg-type]
            oracle_path_length_m=payload["oracle_path_length_m"],  # type: ignore[arg-type]
            seed=payload["seed"],  # type: ignore[arg-type]
            human_review=HumanReview.from_dict(review),
            classification_evidence=evidence,
        )


@dataclass(frozen=True)
class BenchmarkManifest:
    schema_version: str
    benchmark_version: str
    action_protocol_id: str
    localization_source: str
    episodes: tuple[BenchmarkEpisode, ...]
    manifest_sha256: str | None = None

    def __post_init__(self) -> None:
        if self.schema_version != BENCHMARK_SCHEMA_VERSION:
            raise ValueError(f"unsupported benchmark manifest schema: {self.schema_version!r}")
        if self.benchmark_version != BENCHMARK_VERSION:
            raise ValueError(f"unsupported benchmark version: {self.benchmark_version!r}")
        if self.action_protocol_id != ACTION_PROTOCOL_ID:
            raise ValueError(f"unsupported action protocol: {self.action_protocol_id!r}")
        if self.localization_source != LOCALIZATION_SOURCE:
            raise ValueError(f"unsupported localization source: {self.localization_source!r}")

        episodes = tuple(self.episodes)
        if not episodes:
            raise ValueError("benchmark manifest must contain at least one episode")
        if not all(isinstance(episode, BenchmarkEpisode) for episode in episodes):
            raise ValueError("episodes must contain BenchmarkEpisode records")
        episode_ids = [episode.episode_id for episode in episodes]
        if len(set(episode_ids)) != len(episode_ids):
            raise ValueError("duplicate episode_id in benchmark manifest")
        if self.manifest_sha256 is not None:
            _require_sha256(self.manifest_sha256, "manifest_sha256")
        object.__setattr__(self, "episodes", episodes)

    def to_dict(self, *, include_hash: bool = True) -> dict[str, object]:
        payload: dict[str, object] = {
            "schema_version": self.schema_version,
            "benchmark_version": self.benchmark_version,
            "action_protocol_id": self.action_protocol_id,
            "localization_source": self.localization_source,
            "episodes": [episode.to_dict() for episode in self.episodes],
        }
        if include_hash and self.manifest_sha256 is not None:
            payload["manifest_sha256"] = self.manifest_sha256
        return payload

    def computed_sha256(self) -> str:
        return hashlib.sha256(_canonical_json_bytes(self.to_dict(include_hash=False))).hexdigest()

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "BenchmarkManifest":
        episode_payloads = payload.get("episodes")
        if not isinstance(episode_payloads, list):
            raise ValueError("episodes must be an array")
        episodes: list[BenchmarkEpisode] = []
        for episode in episode_payloads:
            if not isinstance(episode, Mapping):
                raise ValueError("each episode must be an object")
            episodes.append(BenchmarkEpisode.from_dict(episode))
        return cls(
            schema_version=payload["schema_version"],  # type: ignore[arg-type]
            benchmark_version=payload["benchmark_version"],  # type: ignore[arg-type]
            action_protocol_id=payload["action_protocol_id"],  # type: ignore[arg-type]
            localization_source=payload["localization_source"],  # type: ignore[arg-type]
            episodes=tuple(episodes),
            manifest_sha256=payload.get("manifest_sha256"),  # type: ignore[arg-type]
        )


def freeze_manifest(manifest: BenchmarkManifest, output_path: str | Path) -> BenchmarkManifest:
    """Approve, hash, and exclusively create a canonical benchmark manifest."""

    if manifest.manifest_sha256 is not None:
        raise ValueError("manifest is already frozen")
    unapproved = [
        episode.episode_id
        for episode in manifest.episodes
        if episode.human_review.status != "approved"
    ]
    if unapproved:
        raise ValueError(
            "all benchmark tasks must be approved before freeze; "
            f"not approved: {', '.join(unapproved)}"
        )

    frozen = replace(manifest, manifest_sha256=manifest.computed_sha256())
    output = Path(output_path)
    if output.exists():
        raise FileExistsError(f"refusing to replace frozen benchmark manifest: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream:
        stream.write(_canonical_json_bytes(frozen.to_dict()))
        stream.write(b"\n")
    return frozen


def load_manifest(path: str | Path) -> BenchmarkManifest:
    """Load a frozen manifest and reject any silent content modification."""

    manifest_path = Path(path)
    with manifest_path.open("r", encoding="utf-8") as stream:
        payload: Any = json.load(stream)
    if not isinstance(payload, Mapping):
        raise ValueError("benchmark manifest root must be an object")
    manifest = BenchmarkManifest.from_dict(payload)
    if manifest.manifest_sha256 is None:
        raise ValueError("frozen benchmark manifest is missing manifest_sha256")
    actual = manifest.computed_sha256()
    if actual != manifest.manifest_sha256:
        raise ValueError(
            "manifest SHA256 mismatch: "
            f"expected {manifest.manifest_sha256}, got {actual}"
        )
    return manifest
