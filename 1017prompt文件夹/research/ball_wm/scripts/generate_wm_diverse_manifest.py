"""Freeze the bounded 12-sequence stereo motion collection before any outcomes.

This writes configuration only; it never starts Isaac or training. Re-running
is byte-identical, and an existing frozen file cannot be silently overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import shlex

RUN_NAME = "wm_aux30_diverse_20260922_v1"
GENERATOR_SEED = 2026092201
PHYSICS_HZ = 50
STEREO_HZ = 10
TOTAL_TICKS = 1200
PYTHON = "/home/an/miniconda3/envs/ball_wm/bin/python"

# Each ordered request program is independently specified before collection.
# Tuples are (main rad/s, auxiliary degrees); durations are seeded below.
# Common [0.7, +15 degrees] follows four deliberately different histories.
PROGRAMS = (
    (1, "forward_history", 0, (
        (.25, 0), (1., 30), (.7, 15), (.55, -20), (0., 0), (-.65, -10))),
    (1, "reverse_history", 45, (
        (-.7, -20), (.7, 15), (.3, 5), (0., -15), (-1., 25), (0., 0))),
    (1, "stop_history", -60, (
        (.6, -25), (0., 0), (.7, 15), (1., -30), (.2, -5), (-.55, 18))),
    (1, "curvature_sweep", 120, (
        (.55, -28), (.55, -8), (.55, 18), (.55, 30), (0., 0), (-.25, -5))),
    (2, "low_speed_history", -30, (
        (-.3, 8), (0., -15), (.25, -8), (.7, 15), (-.9, 30), (0., 0))),
    (2, "reverse_speed_steps", 75, (
        (-.95, 0), (-.6, -30), (-.2, 12), (0., 0), (.9, -25), (.55, 5))),
    (2, "forward_speed_steps", -135, (
        (.2, 5), (.6, 18), (1., 30), (.3, -8), (0., 0), (-.7, -25))),
    (2, "double_reversal", 165, (
        (.85, 25), (-.85, -25), (.45, -10), (0., 0), (-.45, 10), (.95, 0))),
    (4, "mixed_braking", 25, (
        (.3, -18), (-.6, 28), (0., 10), (.95, -5), (.5, 22), (0., -25), (-.25, 0))),
    (4, "reverse_curve_exchange", -105, (
        (-.9, 5), (-.35, -22), (.65, 28), (.2, 0), (-.65, -12), (0., 18))),
    (5, "stop_curve_restart", 100, (
        (0., -8), (.8, 25), (.35, -28), (-.75, 15), (0., 0), (.6, -5))),
    (5, "reverse_brake_sweep", -165, (
        (-.2, -5), (-1., 20), (0., -30), (.55, -15), (.95, 10), (-.45, 28))),
)


def encode_json(value: object) -> bytes:
    return (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def make_segments(requests: tuple, seed: int) -> list[dict]:
    """Allocate seeded 0.1 s quanta above physical transition/hold minima."""
    durations = []
    previous_main = previous_aux_deg = 0.
    for main, aux_deg in requests:
        # At least 0.6 s remains after the configured target slew completes.
        minimum_seconds = max(2.5, abs(main - previous_main) / .5 + .6,
                              abs(aux_deg - previous_aux_deg) / 30 + .6)
        if main < 0:
            minimum_seconds = max(minimum_seconds, 4.2)
        elif main == 0:
            minimum_seconds = max(minimum_seconds, 3.2)
        durations.append(5 * math.ceil((minimum_seconds * PHYSICS_HZ - 1e-8) / 5))
        previous_main, previous_aux_deg = main, aux_deg
    if sum(durations) > TOTAL_TICKS:
        raise ValueError("motion program exceeds the fixed 24-second budget")
    rng = random.Random(seed)
    for _ in range((TOTAL_TICKS - sum(durations)) // 5):
        durations[rng.randrange(len(durations))] += 5
    segments = []
    for index, ((main, aux_deg), steps) in enumerate(zip(requests, durations)):
        segments.append({
            "name": f"segment_{index + 1:02d}",
            "steps": steps,
            "seconds": steps / PHYSICS_HZ,
            "request": [main, math.radians(aux_deg)],
            "request_main_rad_s": main,
            "request_aux_deg": aux_deg,
        })
    return segments


def validate_bundle(files: dict) -> None:
    """Reject budget, split, bounded-action, or accidental template leakage."""
    manifest = files["manifest.json"]
    jobs = manifest["jobs"]
    if len(jobs) != 12:
        raise ValueError("exactly 12 frozen jobs are required")
    request_hashes, orders, durations_by_split = set(), set(), {"train": set(), "val": set(), "test": set()}
    scenes_by_split = {key: set(value) for key, value in manifest["scene_split"].items()}
    if sum(map(len, scenes_by_split.values())) != len(set.union(*scenes_by_split.values())):
        raise ValueError("scene split overlap")
    for job in jobs:
        segments = files[Path(job["command_sequence"]).name]["segments"]
        if sum(s["steps"] for s in segments) != TOTAL_TICKS:
            raise ValueError("sequence duration differs from 1200 ticks")
        if job["scene"] not in scenes_by_split[job["split"]]:
            raise ValueError("job does not belong to declared whole-scene split")
        requests = tuple(tuple(s["request"]) for s in segments)
        mirror = tuple((main, -aux) for main, aux in requests)
        order = min(requests, mirror)
        if order in orders or job["request_array_sha256"] in request_hashes:
            raise ValueError("duplicate request array or mirrored request ordering")
        orders.add(order)
        request_hashes.add(job["request_array_sha256"])
        durations_by_split[job["split"]].add(tuple(s["steps"] for s in segments))
        for segment in segments:
            main, aux = segment["request"]
            if segment["steps"] % 5 or segment["steps"] < 125:
                raise ValueError("segment must be aligned and at least 2.5 seconds")
            if not (-1 <= main <= 1 and abs(aux) <= math.pi / 6):
                raise ValueError("request exceeds bounded action protocol")
    if durations_by_split["train"] & (durations_by_split["val"] | durations_by_split["test"]):
        raise ValueError("held-out sequence duplicates a training duration program")


def build_bundle(workspace: Path, config_dir: Path | None = None) -> dict:
    workspace = Path(workspace).resolve()
    config_dir = (Path(config_dir) if config_dir else workspace / "research/ball_wm/configs/wm_aux30_diverse").resolve()
    base_configs = workspace / "research/ball_wm/configs"
    pilot = json.loads((base_configs / "wm_aux30_pilot_manifest.json").read_text())
    scene_split = pilot["scene_split"]
    scene_by_number = {int(Path(scene).parent.name.split("_")[-1]): scene
                       for scenes in scene_split.values() for scene in scenes}
    dataset_root = workspace / "research/ball_wm/data" / RUN_NAME
    run_root = workspace / "research/ball_wm/runs" / RUN_NAME
    files = {"scene_split.json": scene_split}
    jobs = []
    shell_commands = ["#!/usr/bin/env bash", "set -euo pipefail",
                      f"cd {shlex.quote(str(workspace))}",
                      "# Fixed manifest: one attempt per job; no outcome-driven retries or additions."]
    for index, (scene_number, name, yaw, requests) in enumerate(PROGRAMS):
        job_id = f"easy_{scene_number}_{name}"
        split = "train" if scene_number in (1, 2) else "val" if scene_number == 4 else "test"
        duration_seed = GENERATOR_SEED + index
        spawn_seed = 202609220 + index
        segments = make_segments(requests, duration_seed)
        sequence = {
            "schema": "wm_diverse_requested_sequence_v1",
            "id": job_id,
            "physics_hz": PHYSICS_HZ,
            "stereo_hz": STEREO_HZ,
            "duration_seed": duration_seed,
            "total_steps": TOTAL_TICKS,
            "segments": segments,
        }
        filename = f"{job_id}.json"
        files[filename] = sequence
        output = dataset_root / f"scene_easy_{scene_number}" / name
        sequence_path = config_dir / filename
        expanded = [segment["request"] for segment in segments for _ in range(segment["steps"])]
        command = [PYTHON, "research/ball_wm/scripts/collect_data_dubins.py",
                   "--scene", scene_by_number[scene_number], "--num_episodes", "1",
                   "--episode_len", str(TOTAL_TICKS), "--seed", str(spawn_seed),
                   "--initial_yaw_deg", str(yaw), "--command_sequence", str(sequence_path),
                   "--main_accel_rad_s2", "0.5", "--aux_rate_deg_s", "30",
                   "--auxiliary_velocity_limit_rad_s", "0.30",
                   "--no_save_depth", "--no_save_elevation", "--save_preview",
                   "--audit_after_episode", "--save_dir", str(output)]
        jobs.append({
            "id": job_id, "split": split, "scene": scene_by_number[scene_number],
            "output": str(output), "seed": spawn_seed, "duration_seed": duration_seed,
            "initial_yaw_deg": yaw, "command_sequence": str(sequence_path),
            "command": command, "command_sequence_sha256": sha256(encode_json(sequence)),
            "request_array_sha256": sha256(encode_json(expanded)),
            "expected_physics_ticks": TOTAL_TICKS, "expected_stereo_frames": 240,
            "segment_steps": [s["steps"] for s in segments],
        })
        shell_commands.extend([f"# {split}: {job_id}", shlex.join(command)])
    source_names = ("data_contract.yaml", "wm_aux30_pilot_manifest.json",
                    "wm_aux30_pilot_scene_split.json", "wm_aux30_pilot_left.json", "wm_aux30_pilot_right.json")
    files["manifest.json"] = {
        "schema": "wm_diverse_collection_manifest_v1", "run_name": RUN_NAME,
        "purpose": "Frozen bounded diverse-motion stereo collection for one small world-model training/evaluation run",
        "frozen_before_collection": True,
        "generation_seed": GENERATOR_SEED,
        "duration_algorithm": "Per-program seeded 5-tick allocation after main/aux slew minima; no outcome-dependent adaptation",
        "max_sequences": 12, "seconds_per_sequence": 24,
        "physics_ticks_per_sequence": TOTAL_TICKS, "stereo_frames_per_sequence": 240,
        "state_hz": PHYSICS_HZ, "stereo_hz": STEREO_HZ,
        "action_protocol": "ball_action_v2_aux30", "main_target_rate": .5,
        "main_request_range_rad_s": [-1, 1], "aux_request_range_deg": [-30, 30],
        "aux_target_rate_deg_s": 30, "aux_supervisor_rad_s": .3,
        "dataset_root": str(dataset_root), "workspace": str(workspace), "run_root": str(run_root),
        "scene_split": scene_split, "jobs": jobs,
        "asset_hashes": {scene: sha256(Path(scene).read_bytes()) for scenes in scene_split.values() for scene in scenes},
        "source_config_hashes": {str(base_configs / name): sha256((base_configs / name).read_bytes()) for name in source_names},
        "generator": str(Path(__file__).resolve()),
        "generator_sha256": sha256(Path(__file__).read_bytes()),
        "golden_excluded": ["hard_0", "benchmark_e1", "benchmark_e2"],
        "split_policy": "Whole scenes and whole sequences: train easy_1/easy_2 (4 each), val easy_4 (2), test easy_5 (2)",
        "collection_policy": {"attempts_per_job": 1, "outcome_triggered_retries": False,
                              "outcome_triggered_extra_scans": False, "intra_sequence_resets": False,
                              "save_depth": False, "save_elevation": False,
                              "save_rgb_and_labels": True, "seeded_random_spawn": True},
        "common_followup": {"request": [.7, math.radians(15)],
                            "training_sequences": ["easy_1_forward_history", "easy_1_reverse_history",
                                                   "easy_1_stop_history", "easy_2_low_speed_history"],
                            "interpretation": "Identical follow-up request after distinct uninterrupted physical histories; verify actual states after collection"},
        "test_scope": "Held out from this training run only; easy scenes have appeared in earlier research.",
    }
    files["commands.sh"] = "\n".join(shell_commands) + "\n"
    validate_bundle(files)
    return files


def write_bundle(workspace: Path, config_dir: Path | None = None) -> dict:
    config_dir = (Path(config_dir) if config_dir else Path(workspace) / "research/ball_wm/configs/wm_aux30_diverse").resolve()
    files = build_bundle(workspace, config_dir)
    encoded = {name: value.encode("utf-8") if isinstance(value, str) else encode_json(value)
               for name, value in files.items()}
    # Preflight the entire bundle before writing any files.
    for name, data in encoded.items():
        path = config_dir / name
        if path.exists() and path.read_bytes() != data:
            raise ValueError(f"refusing to change frozen configuration: {path}")
    config_dir.mkdir(parents=True, exist_ok=True)
    for name, data in encoded.items():
        (config_dir / name).write_bytes(data)
    return files["manifest.json"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--config-dir", type=Path)
    args = parser.parse_args()
    manifest = write_bundle(args.workspace, args.config_dir)
    print(json.dumps({"run_name": manifest["run_name"], "jobs": len(manifest["jobs"]),
                      "unique_request_arrays": len({j["request_array_sha256"] for j in manifest["jobs"]}),
                      "physics_ticks": sum(j["expected_physics_ticks"] for j in manifest["jobs"]),
                      "stereo_frames": sum(j["expected_stereo_frames"] for j in manifest["jobs"])}, indent=2))


if __name__ == "__main__":
    main()
