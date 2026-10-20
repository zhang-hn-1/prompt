"""Measure one frozen primitive in the unchanged hard-0 scene via formal execute."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import sys
import traceback
from pathlib import Path

import numpy as np

from primitive_measurement import HORIZON_S, PRIMITIVES, local_outcome, primitive_block


HERE = Path(__file__).resolve().parent
VISION = HERE.parent
PROJECT = Path("/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm")
ROLL = VISION.parents[1] / "oracle_feasible_route_20260929_v9/gated_speed_012"
OLD = VISION / "bilateral_depth_h4/episode_hard_0_e1_0004"
sys.path.insert(0, str(ROLL))
from roll_pid import bank_from_quaternion, install_runtime  # noqa: E402
from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig  # noqa: E402
from benchmark.manifest import load_manifest  # noqa: E402
from training.action_protocol import load_action_protocol  # noqa: E402


def write_json(path: Path, data: dict) -> None:
    def encode(value):
        if isinstance(value, (np.ndarray, np.generic)):
            return value.tolist()
        if isinstance(value, Path):
            return str(value)
        raise TypeError(type(value).__name__)
    path.write_text(json.dumps(data, indent=2, default=encode, allow_nan=False) + "\n")


def state(step) -> dict:
    o = step.observation
    p = o.proprio_history[-1]
    q = o.quaternion_world_wxyz_history[-1]
    return {"position_world_xy": o.position_world_xy.copy(),
            "root_quaternion_world_wxyz": q.copy(),
            "yaw_world_rad": o.yaw_world_rad,
            "bank_rad": bank_from_quaternion(q),
            "linear_velocity_world_m_s": p[:3].copy(),
            "angular_velocity_world_rad_s": p[3:6].copy(),
            "joint_position_rad": p[9:12].copy(),
            "joint_velocity_rad_s": p[12:15].copy(),
            "applied_action": p[15:17].copy(),
            "simulation_time_s": step.simulation_time_s,
            "physics_step_index": step.physics_step_index,
            "runtime_clearance_m": step.truth_clearance_m,
            "physical_contact": step.physical_contact,
            "collision": step.collision}


def initial_execution(runtime, step) -> dict:
    s = state(step)
    smoother = runtime._action_smoother
    pid = smoother.pid
    return {**{k: np.asarray(v).tolist() for k, v in s.items()
               if k in ("position_world_xy", "root_quaternion_world_wxyz",
                         "linear_velocity_world_m_s", "angular_velocity_world_rad_s",
                         "joint_position_rad", "joint_velocity_rad_s", "applied_action")},
            "smoother_current": smoother.current_action.tolist(),
            "smoother_last_target": smoother.last_smoothed_target.tolist(),
            "pid_integral_rate_blocked_previous_available":
                [pid.integral_output, pid.rate, float(smoother.blocked),
                 float(pid.previous_bank is not None)]}


def save_stereo_depth(runtime, step, output: Path, block: int) -> None:
    from bev_ground_truth import read_camera_depth
    from collect_data_dubins import LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH, get_cam_pose
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    position, quaternion = runtime._robot.get_world_pose()
    dest = output / "offline_depth"
    dest.mkdir(exist_ok=True)
    np.savez_compressed(dest / f"frame_{block:03d}.npz",
                        left_rgb=step.observation.stereo_left_rgb,
                        right_rgb=step.observation.stereo_right_rgb,
                        left_depth_m=read_camera_depth(runtime._left_camera),
                        right_depth_m=read_camera_depth(runtime._right_camera),
                        left_camera_usd_world=get_cam_pose(stage, LEFT_CAMERA_PATH),
                        right_camera_usd_world=get_cam_pose(stage, RIGHT_CAMERA_PATH),
                        root_position_world_m=np.asarray(position),
                        root_quaternion_world_wxyz=np.asarray(quaternion),
                        simulation_time_s=step.simulation_time_s)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", required=True, choices=("b0_replay", *PRIMITIVES))
    args = parser.parse_args()
    output = HERE / args.case
    output.mkdir(exist_ok=False)
    manifest_path = PROJECT / "configs/benchmark_v1/hard_0_dev_manifest.json"
    episode = next(e for e in load_manifest(manifest_path).episodes
                   if e.episode_id == "hard_0_e1_0004")
    protocol_path = PROJECT / "configs/data_contract.yaml"
    protocol = load_action_protocol(protocol_path)
    gains = json.loads((ROLL / "selected_candidate.json").read_text())["gains"]
    config = IsaacRuntimeConfig(
        scene_root=Path("/home/an/project/NavDP/assets/scenes"),
        contract_path=protocol_path, gui=False, record_video=True,
        occupancy_path=PROJECT / "data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz",
        lighting_profile="fixed_camera_fill_v1", collision_mode="physical_contact_v2",
        diagnostic_physics_trace=True)
    runtime_class = install_runtime(IsaacBenchmarkRuntime, **gains,
                                    output=output / "feedback")
    runtime = runtime_class(config)
    write_json(output / "identity.json", {
        "scope": "offline diagnostic primitive; not formal navigation",
        "case": args.case, "episode_id": episode.episode_id,
        "horizon_s": HORIZON_S, "formal_navigation_budget_s": 35.0,
        "protocol": protocol.protocol_id, "runtime_config": asdict(config),
        "roll_pid_gains": gains,
        "source_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in (HERE / "PROTOCOL.md", HERE / "run_one.py",
                                    HERE / "primitive_measurement.py", ROLL / "roll_pid.py",
                                    PROJECT / "benchmark/isaac_runtime.py", protocol_path,
                                    manifest_path)}})
    if args.case == "b0_replay":
        with np.load(OLD / "actions.npz") as a:
            requests = a["action_blocks"].copy()
    else:
        action = primitive_block(args.case, controls_per_block=protocol.controls_per_block)
        if not np.array_equal(action, protocol.clip_block(action)):
            raise ValueError("frozen request lies outside formal protocol")
        requests = np.tile(action, (60, 1))
    rows = []
    executed = []
    return_code = 1
    try:
        step = runtime.reset(episode)
        rows.append(state(step))
        write_json(output / "initial_execution.json", initial_execution(runtime, step))
        if step.physical_contact or not runtime.contact_measurement_available:
            raise RuntimeError("invalid initial contact measurement")
        save_stereo_depth(runtime, step, output, 0)
        for index, request in enumerate(requests, start=1):
            step = runtime.execute(request)
            executed.append(request.copy())
            rows.append(state(step))
            if index % 10 == 0 or step.physical_contact or index == len(requests):
                save_stereo_depth(runtime, step, output, index)
            if step.physical_contact:
                break
        trace = {key: np.asarray([r[key] for r in rows]) for key in rows[0]}
        np.savez_compressed(output / "states.npz", **trace)
        np.savez_compressed(output / "physics.npz", **runtime.physics_diagnostics)
        np.savez_compressed(output / "requests.npz", action_blocks=np.asarray(executed))
        contacts = runtime.collision_contact_samples
        write_json(output / "contact_samples.json", {"samples": contacts})
        available = runtime.contact_measurement_available and all(s["available"] for s in contacts)
        hit = any(s["contact"] for s in contacts)
        xy = trace["position_world_xy"]
        displacement = float(np.linalg.norm(xy[-1] - xy[0]))
        yaw = float(trace["yaw_world_rad"][0])
        delta = xy[-1] - xy[0]
        initial_frame_delta = np.array([[np.cos(yaw), np.sin(yaw)],
                                        [-np.sin(yaw), np.cos(yaw)]]) @ delta
        goal = np.asarray(episode.goal_position_world_m[:2])
        write_json(output / "result.json", {
            "case": args.case, "runtime_completed": True,
            "local_outcome": local_outcome(float(step.simulation_time_s), displacement,
                                            any_contact=hit, measurement_available=available),
            "elapsed_s": float(step.simulation_time_s), "executed_blocks": len(executed),
            "physical_contact": hit, "contact_available": available,
            "first_contact_time_s": next((s["simulation_time_s"] for s in contacts if s["contact"]), None),
            "net_displacement_m": displacement,
            "displacement_initial_forward_left_m": initial_frame_delta,
            "goal_distance_decrease_m": float(np.linalg.norm(xy[0] - goal) - np.linalg.norm(xy[-1] - goal)),
            "path_length_m": float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
            "endpoint_world_xy": xy[-1],
            "max_abs_bank_deg": float(np.rad2deg(np.abs(trace["bank_rad"])).max()),
            "candidate_is_stop": args.case == "stop"})
        import imageio.v2 as imageio
        for kind, frames in runtime.captured_frames.items():
            imageio.mimsave(output / f"{kind}.mp4", frames, fps=10, macro_block_size=2)
        return_code = 0
        print("RESULT", (output / "result.json").read_text(), flush=True)
    except BaseException:
        write_json(output / "failure.json", {"traceback": traceback.format_exc()})
        raise
    finally:
        runtime.close(return_code=return_code)


if __name__ == "__main__":
    main()
