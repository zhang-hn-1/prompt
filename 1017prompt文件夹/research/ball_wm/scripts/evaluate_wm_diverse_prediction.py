"""Causal offline pose/BEV evaluation with a fixed five-frame window stride.

Only observed stereo/proprioception and supplied future requests enter the
model. Future poses and BEVs are labels; future RGB images are never opened.
This command evaluates one externally selected checkpoint, not navigation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import torch
from torch.utils.data import DataLoader

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from training.ablation_protocol import scene_indices
from training.action_protocol import require_action_protocol
from training.ball_wm_dataset import (
    BallWMDataset, _load_episode, _load_image, _normalize_image, _proprio17,
)
from training.joint_jepa import JointModelConfig, build_joint_model
from training.pose_utils import planar_yaw_from_wxyz, relative_planar_pose
from training.proprioception import checkpoint_velocity_frame

HISTORY = 5
HORIZON = 5
WINDOW_STRIDE = 5
FRAME_DT = 0.1
SCORED_STEPS = (1, 3, 5)
PREFIX_STEPS = 3
PREFIX_ATOL = 1e-5
MODEL_NAMES = ("model", "zero_motion", "constant_velocity")
INPUT_KEYS = ("pixels_left", "pixels_right", "proprio", "action")


def _wrapped(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def select_window_indices(dataset, manifest: dict, split: str) -> list[int]:
    """Use every complete start=0,5,10,... window independently per episode."""

    if split not in ("val", "test"):
        raise ValueError("evaluation split must be val or test")
    partitions = scene_indices(dataset.sample_scenes, manifest)
    indices = partitions[1 if split == "val" else 2]
    selected = [index for index in indices if dataset.samples[index][1] % WINDOW_STRIDE == 0]
    if not selected:
        raise ValueError(f"split {split!r} has no windows")
    return selected


def motion_baselines(world_velocity, current_yaw, previous_yaw, observed_dt, future_dt):
    """Extrapolate current world vXY and yaw rate from two observed frames.

    XY is expressed in the last observed base frame, exactly like
    relative_planar_pose. World velocity is held constant, not rotated along
    a hypothetical arc. World omega_z is not assumed to equal root yaw rate.
    """

    velocity = np.asarray(world_velocity, dtype=np.float64)
    times = np.asarray(future_dt, dtype=np.float64)
    if velocity.shape != (3,) or times.ndim != 1 or not np.isfinite(velocity).all() or not np.isfinite(times).all():
        raise ValueError("baseline velocity/times must be finite with shapes (3,) and (H,)")
    if not np.isfinite([current_yaw, previous_yaw, observed_dt]).all() or observed_dt <= 0 or (times <= 0).any():
        raise ValueError("baseline orientations and positive time intervals must be finite")
    zero = np.zeros((times.size, 4), dtype=np.float64)
    zero[:, 3] = 1.0
    cosine, sine = math.cos(current_yaw), math.sin(current_yaw)
    local_velocity = np.array([cosine * velocity[0] + sine * velocity[1], -sine * velocity[0] + cosine * velocity[1]])
    yaw_rate = float(_wrapped(current_yaw - previous_yaw)) / observed_dt
    cv = np.column_stack((times[:, None] * local_velocity, np.sin(times * yaw_rate), np.cos(times * yaw_rate)))
    return zero, cv


class CausalEvaluationDataset(torch.utils.data.Dataset):
    """Reuse training indices/preprocessing while loading only context RGB."""

    def __init__(self, source: BallWMDataset, indices: list[int]):
        if source.config.history_frames != HISTORY or source.config.rollout_horizon != HORIZON:
            raise ValueError("evaluation requires history=5 and rollout_horizon=5")
        if source.config.action_source != "commanded_action" or source.config.action_block != 5:
            raise ValueError("evaluation requires commanded_action with five controls per block")
        self.source = source
        self.indices = indices

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, index):
        episode_index, start = self.source.samples[self.indices[index]]
        episode = self.source.episode_paths[episode_index]
        loaded = _load_episode(str(episode))
        states, metadata = loaded["states"], loaded["metadata"]
        block = self.source.controls_per_block
        context_frames = np.arange(start, start + HISTORY)
        context_states = block * (context_frames + 1) - 1
        anchor = int(context_states[-1])
        targets = anchor + block * np.arange(1, HORIZON + 1)
        timestamps = np.asarray(states["state_timestamp"], dtype=np.float64)
        future_dt = timestamps[targets] - timestamps[anchor]
        observed_dt = float(timestamps[anchor] - timestamps[context_states[-2]])
        if not np.allclose(future_dt, np.arange(1, HORIZON + 1) * FRAME_DT, atol=1e-6, rtol=0):
            raise ValueError(f"{episode}: future state timestamps do not match 10 Hz horizons")
        if not math.isclose(observed_dt, FRAME_DT, abs_tol=1e-6, rel_tol=0):
            raise ValueError(f"{episode}: observed state timestamps do not match 10 Hz")
        frames = np.arange(start, start + HISTORY + HORIZON)
        aligned_states = block * (frames + 1) - 1
        if not np.array_equal(states["stereo_physics_step"][frames], states["physics_step"][aligned_states]):
            raise ValueError(f"{episode}: stereo and state physics steps are misaligned")

        def images(side):
            values = [_load_image(episode / f"{side}_{frame:06d}.png", self.source.config.image_size) for frame in context_frames]
            return torch.stack([_normalize_image(value) for value in values] if self.source.config.normalize else values)

        actions = np.asarray(states["commanded_action"], dtype=np.float32)
        context_actions = np.stack([actions[state + 1 : state + 1 + block].reshape(-1) for state in context_states])
        future_actions = actions[anchor + 1 : anchor + 1 + block * HORIZON].reshape(HORIZON, 2 * block)
        poses = np.stack([
            relative_planar_pose(states["pos"][anchor], states["quat"][anchor], states["pos"][target], states["quat"][target])
            for target in targets
        ])
        current_yaw = planar_yaw_from_wxyz(states["quat"][anchor])
        previous_yaw = planar_yaw_from_wxyz(states["quat"][context_states[-2]])
        zero, cv = motion_baselines(states["lin_vel"][anchor], current_yaw, previous_yaw, observed_dt, future_dt)
        bevs = []
        for frame in range(start + HISTORY, start + HISTORY + HORIZON):
            with Image.open(episode / f"geometry_bev_{frame:06d}.png") as image:
                bevs.append(np.asarray(image.convert("L"), dtype=np.int64))
        return {
            "pixels_left": images("left"), "pixels_right": images("right"),
            "proprio": torch.from_numpy(np.stack([
                _proprio17(states, int(state), velocity_frame=self.source.config.proprio_velocity_frame)
                for state in context_states
            ])),
            "action": torch.from_numpy(context_actions),
            "future_action": torch.from_numpy(future_actions.copy()),
            "future_relative_pose": torch.from_numpy(poses),
            "future_bev": torch.from_numpy(np.stack(bevs)),
            "baseline_zero": torch.from_numpy(zero.astype(np.float32)),
            "baseline_cv": torch.from_numpy(cv.astype(np.float32)),
            "episode": str(episode.resolve()), "scene": str(metadata["scene"]),
            "start_frame": int(start),
        }


def pose_errors(predicted, target):
    predicted, target = np.asarray(predicted, dtype=np.float64), np.asarray(target, dtype=np.float64)
    if predicted.shape != (4,) or target.shape != (4,) or not np.isfinite([predicted, target]).all():
        raise ValueError("pose prediction and target must be finite (4,) vectors")
    predicted_yaw = np.arctan2(predicted[2], predicted[3])
    target_yaw = np.arctan2(target[2], target[3])
    return {
        "displacement_error_m": float(np.linalg.norm(predicted[:2] - target[:2])),
        "yaw_abs_error_rad": float(abs(_wrapped(predicted_yaw - target_yaw))),
    }


def summarize_rows(rows):
    if not rows:
        raise ValueError("evaluation has no windows")
    result = {"window_count": len(rows), "per_horizon": {}}
    for step in SCORED_STEPS:
        key = f"{step * FRAME_DT:.1f}"
        summary = {}
        for name in MODEL_NAMES:
            errors = [row["per_horizon"][key][name] for row in rows]
            xy = np.array([error["displacement_error_m"] for error in errors])
            yaw = np.array([error["yaw_abs_error_rad"] for error in errors])
            summary[name] = {
                "displacement_error_m_mean": float(xy.mean()),
                "displacement_error_m_rmse": float(np.sqrt(np.mean(xy ** 2))),
                "displacement_error_m_p95": float(np.percentile(xy, 95)),
                "yaw_abs_error_rad_mean": float(yaw.mean()),
                "yaw_abs_error_deg_mean": float(np.degrees(yaw.mean())),
            }
        counts = [row["per_horizon"][key]["recursive_occupied_bev"] for row in rows]
        if all(count is None for count in counts):
            summary["recursive_occupied_bev"] = None
        elif any(count is None for count in counts):
            raise ValueError("inconsistent availability of recursive BEV predictions")
        else:
            intersection = sum(count["intersection"] for count in counts)
            union = sum(count["union"] for count in counts)
            summary["recursive_occupied_bev"] = {
                "intersection": intersection, "union": union,
                "occupied_iou": intersection / union if union else None,
            }
        result["per_horizon"][key] = summary
    diagnostics = [row["action_diagnostics"] for row in rows]
    prefix_delta = max(row["prefix_latent_max_abs_delta"] for row in diagnostics)
    result["action_diagnostics"] = {
        "prefix_steps": PREFIX_STEPS, "prefix_atol": PREFIX_ATOL,
        "prefix_latent_max_abs_delta": prefix_delta,
        "prefix_causality_passed": prefix_delta <= PREFIX_ATOL,
        "negated_request_pose_xy_change_m_mean": float(np.mean([row["negated_request_pose_xy_change_m"] for row in diagnostics])),
        "negated_request_pose_yaw_change_rad_mean": float(np.mean([row["negated_request_pose_yaw_change_rad"] for row in diagnostics])),
        "request_change_l1_mean": float(np.mean([row["request_change_l1_mean"] for row in diagnostics])),
        "windows_with_changed_requests": sum(row["request_change_l1_mean"] > 0 for row in diagnostics),
        "interpretation": "prediction sensitivity only; counterfactual requests have no recorded ground truth",
    }
    return result


@torch.inference_mode()
def evaluate_model(model, loader, device="cpu"):
    device = torch.device(device)
    model.eval()
    rows = []
    for batch in loader:
        context = {name: batch[name].to(device) for name in INPUT_KEYS}
        if any(value.shape[1] != HISTORY or not torch.isfinite(value).all() for value in context.values()):
            raise ValueError("model inputs must contain exactly five finite observed frames")
        requests = batch["future_action"].to(device)
        if requests.shape[1:] != (HORIZON, 10) or not torch.isfinite(requests).all():
            raise ValueError("future requests must have finite shape (B,5,10)")
        changed_suffix = requests.clone()
        changed_suffix[:, PREFIX_STEPS:] *= -1
        candidates = torch.stack((requests, -requests, changed_suffix), dim=1)
        embeddings = model.rollout(context, candidates, history_size=HISTORY)
        predicted_poses = model.decode_relative_pose(embeddings).cpu().numpy()
        if predicted_poses.shape != (*candidates.shape[:-1], 4) or not np.isfinite(predicted_poses).all():
            raise ValueError("model must predict finite (B,3,5,4) relative poses")
        if not torch.isfinite(embeddings).all():
            raise ValueError("model predicted nonfinite latent embeddings")
        prefix_delta = (embeddings[:, 0, :PREFIX_STEPS] - embeddings[:, 2, :PREFIX_STEPS]).abs().flatten(1).amax(1).cpu().numpy()
        predicted_bev = None
        if getattr(model, "bev_head", None) is not None:
            logits = model.bev_head(embeddings[:, 0])
            if logits.ndim != 5 or logits.shape[2] != 3 or not torch.isfinite(logits).all():
                raise ValueError("recursive BEV head must produce finite (B,5,3,H,W) logits")
            predicted_bev = logits.argmax(dim=2).cpu().numpy()
        targets = batch["future_relative_pose"].cpu().numpy()
        target_bev = batch["future_bev"].cpu().numpy()
        if targets.shape != predicted_poses[:, 0].shape or not np.isfinite(targets).all():
            raise ValueError("future pose labels have invalid shape or nonfinite values")
        if not np.isin(target_bev, (0, 1, 2)).all() or (predicted_bev is not None and target_bev.shape != predicted_bev.shape):
            raise ValueError("future BEV labels must match recursive predictions and use classes 0,1,2")
        for index in range(requests.size(0)):
            poses = {
                "model": predicted_poses[index, 0],
                "zero_motion": batch["baseline_zero"][index].cpu().numpy(),
                "constant_velocity": batch["baseline_cv"][index].cpu().numpy(),
            }
            per_horizon = {}
            for step in SCORED_STEPS:
                scores = {name: pose_errors(prediction[step - 1], targets[index, step - 1]) for name, prediction in poses.items()}
                counts = None
                if predicted_bev is not None:
                    occupied = predicted_bev[index, step - 1] == 1
                    truth = target_bev[index, step - 1] == 1
                    counts = {"intersection": int((occupied & truth).sum()), "union": int((occupied | truth).sum())}
                scores["recursive_occupied_bev"] = counts
                per_horizon[f"{step * FRAME_DT:.1f}"] = scores
            differences = [pose_errors(predicted_poses[index, 1, step - 1], predicted_poses[index, 0, step - 1]) for step in SCORED_STEPS]
            rows.append({
                "episode": batch["episode"][index], "scene": batch["scene"][index],
                "start_frame": int(batch["start_frame"][index]), "per_horizon": per_horizon,
                "action_diagnostics": {
                    "prefix_latent_max_abs_delta": float(prefix_delta[index]),
                    "negated_request_pose_xy_change_m": float(np.mean([value["displacement_error_m"] for value in differences])),
                    "negated_request_pose_yaw_change_rad": float(np.mean([value["yaw_abs_error_rad"] for value in differences])),
                    "request_change_l1_mean": float((2 * requests[index]).abs().mean().cpu()),
                },
            })
    per_episode = []
    for episode in sorted({row["episode"] for row in rows}):
        selected = [row for row in rows if row["episode"] == episode]
        per_episode.append({
            "episode": episode, "scene": selected[0]["scene"],
            "window_start_frames": [row["start_frame"] for row in selected],
            **summarize_rows(selected),
        })
    return {"aggregate": summarize_rows(rows), "per_episode": per_episode}


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data_root", type=Path, required=True)
    parser.add_argument("--scene_split_manifest", type=Path, required=True)
    parser.add_argument("--split", choices=("val", "test"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--batch_size", type=int, default=2)
    args = parser.parse_args(argv)
    if args.batch_size < 1:
        raise ValueError("batch_size must be positive")
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    config = JointModelConfig(**checkpoint["config"])
    velocity_frame = checkpoint_velocity_frame(checkpoint)
    if config.history_size != HISTORY or config.rollout_horizon != HORIZON or not config.pose_head_enabled:
        raise ValueError("checkpoint requires history=5, horizon=5 and an enabled pose head")
    if config.action_source != "commanded_action" or checkpoint.get("action_source", config.action_source) != config.action_source:
        raise ValueError("checkpoint must consistently declare commanded_action")
    source = BallWMDataset(
        args.data_root, history_frames=HISTORY, num_preds=config.num_preds,
        rollout_horizon=HORIZON, image_size=(config.image_size, config.image_size),
        with_visibility=False, action_source=config.action_source,
        proprio_velocity_frame=velocity_frame,
    )
    require_action_protocol(checkpoint, source.action_protocol_id, artifact="checkpoint")
    manifest = json.loads(args.scene_split_manifest.read_text())
    indices = select_window_indices(source, manifest, args.split)
    dataset = CausalEvaluationDataset(source, indices)
    device = torch.device(args.device)
    model = build_joint_model(config).to(device)
    model.load_state_dict(checkpoint["model"], strict=True)
    report = evaluate_model(model, DataLoader(dataset, batch_size=args.batch_size, shuffle=False, num_workers=0), device)
    report.update({
        "schema_version": "wm_diverse_causal_prediction_v1",
        "checkpoint": str(args.checkpoint.resolve()), "checkpoint_sha256": _sha256(args.checkpoint),
        "checkpoint_epoch": checkpoint.get("epoch"), "data_root": str(args.data_root.resolve()),
        "scene_split_manifest": str(args.scene_split_manifest.resolve()),
        "scene_split_manifest_sha256": _sha256(args.scene_split_manifest),
        "split": args.split, "device": str(device), "action_source": config.action_source,
        "action_protocol_id": source.action_protocol_id,
        "proprio_velocity_frame": velocity_frame,
        "protocol": {
            "history_frames": HISTORY, "rollout_frames": HORIZON, "window_stride_frames": WINDOW_STRIDE,
            "horizons_s": [0.1, 0.3, 0.5], "window_selection": "all complete per-episode starts 0,5,10,...",
            "model_inputs": "five past stereo/proprio frames and requested future actions; no future RGB/proprio encoding",
            "pose_frame": "XY and yaw relative to the final observed base frame",
            "constant_velocity": "hold current world linear velocity; rotate displacement into anchor base; yaw rate is wrapped difference of last two observed yaw values / observed dt",
            "bev": "recursive latent predictions, occupied class 1, full-grid pixel intersection/union; null IoU if union is empty",
            "aggregation": "pose errors averaged over windows; BEV intersection/union pooled over pixels",
            "counterfactuals": "request sign negation; suffix sign negation after step 3; sensitivity only, no counterfactual ground-truth scores",
            "checkpoint_selection": "external; this evaluator never ranks or selects checkpoints",
            "model_limitation": (
                "legacy proprio velocities are world-frame and contain no absolute yaw"
                if velocity_frame == "world" else
                "velocities use measured full base rotation; pose labels remain planar relative yaw; no absolute yaw input"
            ),
        },
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output), "split": args.split, "windows": report["aggregate"]["window_count"]}))
    return report


if __name__ == "__main__":
    main()
