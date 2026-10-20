"""Frozen motion schedules must be diverse, bounded, and reproducible."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "research/ball_wm/scripts/generate_wm_diverse_manifest.py"


def generator():
    assert SCRIPT.exists(), "deterministic diverse manifest generator is missing"
    spec = importlib.util.spec_from_file_location("generate_wm_diverse_manifest", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def bundle(tmp_path):
    return generator().build_bundle(ROOT, tmp_path)


def test_fixed_budget_and_whole_scene_split(tmp_path):
    files = bundle(tmp_path)
    manifest = files["manifest.json"]
    assert manifest["frozen_before_collection"] is True
    assert (manifest["max_sequences"], manifest["seconds_per_sequence"]) == (12, 24)
    assert manifest["state_hz"] == 50 and manifest["stereo_hz"] == 10
    assert [j["split"] for j in manifest["jobs"]].count("train") == 8
    assert [j["split"] for j in manifest["jobs"]].count("val") == 2
    assert [j["split"] for j in manifest["jobs"]].count("test") == 2
    split = files["scene_split.json"]
    assert split == manifest["scene_split"]
    assert len({s for paths in split.values() for s in paths}) == 4
    assert all(j["scene"] in split[j["split"]] for j in manifest["jobs"])
    assert [Path(j["scene"]).parent.name for j in manifest["jobs"]].count("easy_1") == 4
    assert [Path(j["scene"]).parent.name for j in manifest["jobs"]].count("easy_2") == 4
    assert len({j["initial_yaw_deg"] for j in manifest["jobs"]}) >= 6


def test_duration_bounds_and_physical_hold_coverage(tmp_path):
    files = bundle(tmp_path)
    reverse_holds = stop_holds = 0
    common_histories = set()
    for job in files["manifest.json"]["jobs"]:
        sequence = files[Path(job["command_sequence"]).name]
        assert sum(s["steps"] for s in sequence["segments"]) == 1200
        previous_main = previous_aux = 0.0
        for segment in sequence["segments"]:
            main, aux = segment["request"]
            assert segment["steps"] % 5 == 0 and segment["steps"] >= 125
            assert -1 <= main <= 1 and abs(aux) <= math.pi / 6
            duration = segment["steps"] / 50
            assert duration >= abs(main - previous_main) / 0.5 + 0.5 - 1e-9
            if main < 0 and duration >= 4:
                reverse_holds += 1
            if main == 0 and duration >= 3:
                stop_holds += 1
            if job["split"] == "train" and main == 0.7 and math.isclose(aux, math.radians(15)):
                common_histories.add((previous_main, previous_aux))
            previous_main, previous_aux = main, aux
        assert job["expected_physics_ticks"] == 1200
        assert job["expected_stereo_frames"] == 240
    assert reverse_holds >= 8 and stop_holds >= 8
    assert len(common_histories) >= 4


def test_unique_arrays_orders_and_heldout_durations(tmp_path):
    files = bundle(tmp_path)
    expanded = set()
    canonical_orders = set()
    train_durations = set()
    heldout_durations = set()
    for job in files["manifest.json"]["jobs"]:
        segments = files[Path(job["command_sequence"]).name]["segments"]
        requests = tuple(tuple(s["request"]) for s in segments)
        mirrored = tuple((main, -aux) for main, aux in requests)
        canonical_orders.add(min(requests, mirrored))
        expanded.add(tuple(pair for s in segments for pair in [tuple(s["request"])] * s["steps"]))
        durations = tuple(s["steps"] for s in segments)
        (train_durations if job["split"] == "train" else heldout_durations).add(durations)
    assert len(expanded) == len(canonical_orders) == 12
    assert train_durations.isdisjoint(heldout_durations)


def test_commands_and_hashes_are_complete(tmp_path):
    module = generator()
    files = module.build_bundle(ROOT, tmp_path)
    manifest = files["manifest.json"]
    for job in manifest["jobs"]:
        command = job["command"]
        assert command[command.index("--episode_len") + 1] == "1200"
        assert command[command.index("--seed") + 1] == str(job["seed"])
        assert command[command.index("--initial_yaw_deg") + 1] == str(job["initial_yaw_deg"])
        assert command[command.index("--main_accel_rad_s2") + 1] == "0.5"
        assert command[command.index("--aux_rate_deg_s") + 1] == "30"
        assert command[command.index("--auxiliary_velocity_limit_rad_s") + 1] == "0.30"
        assert "--no_save_depth" in command and "--no_save_elevation" in command
        data = module.encode_json(files[Path(job["command_sequence"]).name])
        assert job["command_sequence_sha256"] == hashlib.sha256(data).hexdigest()
    assert manifest["source_config_hashes"]
    assert "hard_0" in manifest["golden_excluded"]


def test_reproducible_and_frozen_against_mutation(tmp_path):
    module = generator()
    assert module.build_bundle(ROOT, tmp_path) == module.build_bundle(ROOT, tmp_path)
    module.write_bundle(ROOT, tmp_path)
    before = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    module.write_bundle(ROOT, tmp_path)
    assert before == {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    changed = tmp_path / "easy_1_forward_history.json"
    changed.write_text("{}\n")
    with pytest.raises(ValueError, match="frozen"):
        module.write_bundle(ROOT, tmp_path)
