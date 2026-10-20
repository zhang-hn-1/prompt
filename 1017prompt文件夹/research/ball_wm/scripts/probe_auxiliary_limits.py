"""Low-speed Isaac Sim acceptance probe for the auxiliary steering joint."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Sequence

import numpy as np

from action_smoothing import (
    DEFAULT_AUXILIARY_KD,
    DEFAULT_AUXILIARY_KP,
    DEFAULT_AUXILIARY_RATE_RAD_S,
    DEFAULT_MAIN_ACCELERATION_RAD_S2,
    DEFAULT_MAIN_DERIVATIVE_GAIN,
    slew_limited_target,
)


MAX_JOINT_RAD = math.radians(30.5)
MAX_TRACKING_ERROR_RAD = math.radians(3.0)
MIN_YAW_DELTA_RAD = math.radians(3.0)
MAX_AUX_SPEED_RAD_S = math.radians(40.0)
MAX_HEIGHT_DEVIATION_M = 0.05
EXPECTED_PROTOCOL_ID = "ball_action_v2_aux30"
EXPECTED_AUXILIARY_LIMIT_RAD = math.radians(30.0)
SETTLE_STEPS = 50
DRIVE_STEPS = 100
MAIN_VELOCITY_RAD_S = 0.5
DEFAULT_MAIN_ACCEL_RAD_S2 = DEFAULT_MAIN_ACCELERATION_RAD_S2
DEFAULT_AUX_KP = DEFAULT_AUXILIARY_KP
DEFAULT_AUX_KD = DEFAULT_AUXILIARY_KD
DEFAULT_AUX_RATE_DEG_S = math.degrees(DEFAULT_AUXILIARY_RATE_RAD_S)
ROBOT_START = np.asarray([0.0, 0.0, 0.45], dtype=np.float64)
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTRACT = SCRIPT_DIR.parent / "configs" / "data_contract.yaml"
ROBOT_USD = (
    SCRIPT_DIR.parents[2]
    / "project"
    / "rl_env_0.4"
    / "rl_env"
    / "rotunbot_wheel_sim_model_0.4"
    / "Rotunbot_wheel.usd"
)
ROBOT_ROOT = "/World/BallRobot"


def _finite_direction_pair(values: Sequence[float], *, name: str) -> np.ndarray:
    pair = np.asarray(values, dtype=np.float64)
    if pair.shape != (2,):
        raise ValueError(f"{name} must have shape (2,), got {pair.shape}")
    if not np.isfinite(pair).all():
        raise ValueError(f"{name} must contain only finite values")
    return pair


def probe_controls_from_contract(contract: object) -> tuple[tuple[str, float], ...]:
    """Validate the V2 physical contract and derive both steering commands."""

    if getattr(contract, "action_protocol_id", None) != EXPECTED_PROTOCOL_ID:
        raise ValueError(f"probe requires action protocol {EXPECTED_PROTOCOL_ID}")
    expected_names = (
        "main_joint_velocity_cmd",
        "auxiliary_joint_angle_cmd",
    )
    if getattr(contract, "action_names", None) != expected_names:
        raise ValueError("probe requires the two-field ball action protocol")
    minimum = _finite_direction_pair(
        getattr(contract, "action_minimum", ()), name="action_minimum"
    )
    maximum = _finite_direction_pair(
        getattr(contract, "action_maximum", ()), name="action_maximum"
    )
    auxiliary_minimum = float(minimum[1])
    auxiliary_maximum = float(maximum[1])
    if not (
        math.isclose(
            auxiliary_minimum,
            -EXPECTED_AUXILIARY_LIMIT_RAD,
            rel_tol=0.0,
            abs_tol=1e-6,
        )
        and math.isclose(
            auxiliary_maximum,
            EXPECTED_AUXILIARY_LIMIT_RAD,
            rel_tol=0.0,
            abs_tol=1e-6,
        )
    ):
        raise ValueError("V2 probe requires symmetric auxiliary bounds of 30 degrees")
    if not minimum[0] <= MAIN_VELOCITY_RAD_S <= maximum[0]:
        raise ValueError("probe main velocity exceeds the action contract")
    return (
        ("positive", auxiliary_maximum),
        ("negative", auxiliary_minimum),
    )


def contact_filter_groups(sensor_paths: Sequence[str]) -> list[list[str]]:
    """Return one empty PhysX filter group for every rigid sensor pattern."""

    paths = list(sensor_paths)
    if not paths or any(not isinstance(path, str) or not path for path in paths):
        raise ValueError("sensor_paths must contain non-empty prim paths")
    return [[] for _ in paths]


def evaluate_probe(
    *,
    commanded_rad: Sequence[float],
    measured_rad: Sequence[float],
    max_abs_measured_rad: Sequence[float],
    yaw_delta_rad: Sequence[float],
    max_abs_roll_rad: Sequence[float],
    max_aux_speed_rad_s: Sequence[float],
    max_height_deviation_m: Sequence[float],
    collision: Sequence[bool],
) -> dict[str, object]:
    """Evaluate the two-direction steering probe against physical thresholds."""

    commanded = _finite_direction_pair(commanded_rad, name="commanded_rad")
    measured = _finite_direction_pair(measured_rad, name="measured_rad")
    max_abs_measured = _finite_direction_pair(
        max_abs_measured_rad, name="max_abs_measured_rad"
    )
    yaw_delta = _finite_direction_pair(yaw_delta_rad, name="yaw_delta_rad")
    max_abs_roll = _finite_direction_pair(max_abs_roll_rad, name="max_abs_roll_rad")
    max_aux_speed = _finite_direction_pair(
        max_aux_speed_rad_s, name="max_aux_speed_rad_s"
    )
    max_height_deviation = _finite_direction_pair(
        max_height_deviation_m, name="max_height_deviation_m"
    )

    collision_pair = np.asarray(collision)
    if collision_pair.shape != (2,):
        raise ValueError(
            f"collision must have shape (2,), got {collision_pair.shape}"
        )
    if collision_pair.dtype.kind not in {"b", "i", "u"}:
        raise ValueError("collision must contain two boolean values")
    collision_pair = collision_pair.astype(bool)

    joint_limit_ok = bool(np.all(np.abs(max_abs_measured) <= MAX_JOINT_RAD))
    tracking_error = np.abs(measured - commanded)
    tracking_error_ok = bool(np.all(tracking_error <= MAX_TRACKING_ERROR_RAD))
    opposite_yaw_sign_ok = bool(yaw_delta[0] * yaw_delta[1] < 0.0)
    yaw_magnitude_ok = bool(np.all(np.abs(yaw_delta) >= MIN_YAW_DELTA_RAD))
    aux_speed_ok = bool(np.all(np.abs(max_aux_speed) <= MAX_AUX_SPEED_RAD_S))
    height_stability_ok = bool(
        np.all(np.abs(max_height_deviation) <= MAX_HEIGHT_DEVIATION_M)
    )
    collision_free = bool(not np.any(collision_pair))

    checks = {
        "joint_limit_ok": joint_limit_ok,
        "tracking_error_ok": tracking_error_ok,
        "opposite_yaw_sign_ok": opposite_yaw_sign_ok,
        "yaw_magnitude_ok": yaw_magnitude_ok,
        "aux_speed_ok": aux_speed_ok,
        "height_stability_ok": height_stability_ok,
        "collision_free": collision_free,
    }
    return {
        "passed": all(checks.values()),
        **checks,
        "roll_diagnostic_only": True,
        "max_abs_roll_rad": max_abs_roll.tolist(),
        "tracking_error_rad": tracking_error.tolist(),
        "thresholds_rad": {
            "max_abs_joint": MAX_JOINT_RAD,
            "max_tracking_error": MAX_TRACKING_ERROR_RAD,
            "min_abs_yaw_delta": MIN_YAW_DELTA_RAD,
            "max_abs_aux_speed_rad_s": MAX_AUX_SPEED_RAD_S,
        },
        "thresholds_m": {"max_height_deviation": MAX_HEIGHT_DEVIATION_M},
    }


def _finite_trace(values: object, *, name: str, columns: int | None = None) -> np.ndarray:
    try:
        trace = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain finite numeric samples") from exc
    expected_ndim = 2 if columns is not None else 1
    if trace.ndim != expected_ndim or (columns is not None and trace.shape[1] != columns):
        shape = "(N,)" if columns is None else f"(N, {columns})"
        raise ValueError(f"{name} must have shape {shape}")
    if trace.shape[0] < 2 or not np.isfinite(trace).all():
        raise ValueError(f"{name} must contain at least two finite samples")
    return trace


def summarize_direction(
    *,
    label: str,
    main_command_rad_s: float,
    main_target_rad_s: Sequence[float],
    auxiliary_command_rad: float,
    joint_target_rad: Sequence[float],
    joint_rad: Sequence[float],
    yaw_rad: Sequence[float],
    roll_rad: Sequence[float],
    joint_velocity_rad_s: Sequence[float],
    base_z_m: Sequence[float],
    vertical_velocity_m_s: Sequence[float],
    contact_force_norm: Sequence[float],
    collision: Sequence[bool],
    position_xy: Sequence[Sequence[float]],
) -> dict[str, object]:
    """Validate and summarize one direction while retaining its full trace."""

    if label not in {"positive", "negative"}:
        raise ValueError("label must be positive or negative")
    scalar_commands = np.asarray(
        [main_command_rad_s, auxiliary_command_rad], dtype=np.float64
    )
    if not np.isfinite(scalar_commands).all():
        raise ValueError("commands must be finite")

    main_target = _finite_trace(main_target_rad_s, name="main_target_rad_s")
    joint_target = _finite_trace(joint_target_rad, name="joint_target_rad")
    joint = _finite_trace(joint_rad, name="joint_rad")
    yaw = _finite_trace(yaw_rad, name="yaw_rad")
    roll = _finite_trace(roll_rad, name="roll_rad")
    joint_velocity = _finite_trace(
        joint_velocity_rad_s, name="joint_velocity_rad_s"
    )
    base_z = _finite_trace(base_z_m, name="base_z_m")
    vertical_velocity = _finite_trace(
        vertical_velocity_m_s, name="vertical_velocity_m_s"
    )
    contact = _finite_trace(contact_force_norm, name="contact_force_norm")
    positions = _finite_trace(position_xy, name="position_xy", columns=2)
    collisions = np.asarray(collision)
    if collisions.ndim != 1 or collisions.shape[0] < 2:
        raise ValueError("collision must have shape (N,) with at least two samples")
    if collisions.dtype.kind not in {"b", "i", "u"}:
        raise ValueError("collision must contain boolean samples")

    lengths = {
        main_target.shape[0],
        joint_target.shape[0],
        joint.shape[0],
        yaw.shape[0],
        roll.shape[0],
        joint_velocity.shape[0],
        base_z.shape[0],
        vertical_velocity.shape[0],
        contact.shape[0],
        collisions.shape[0],
        positions.shape[0],
    }
    if len(lengths) != 1:
        raise ValueError("all direction traces must have equal lengths")
    collision_flags = collisions.astype(bool)
    unwrapped_yaw = np.unwrap(yaw)
    steady_samples = joint[-min(10, joint.shape[0]) :]

    samples = [
        {
            "step": index,
            "main_target_rad_s": float(main_target[index]),
            "joint_target_rad": float(joint_target[index]),
            "joint_rad": float(joint[index]),
            "yaw_rad": float(unwrapped_yaw[index]),
            "roll_rad": float(roll[index]),
            "joint_velocity_rad_s": float(joint_velocity[index]),
            "base_z_m": float(base_z[index]),
            "vertical_velocity_m_s": float(vertical_velocity[index]),
            "contact_force_norm": float(contact[index]),
            "collision": bool(collision_flags[index]),
            "position_xy_m": positions[index].tolist(),
        }
        for index in range(joint.shape[0])
    ]
    return {
        "label": label,
        "main_command_rad_s": float(main_command_rad_s),
        "auxiliary_command_rad": float(auxiliary_command_rad),
        "measured_auxiliary_rad": float(np.median(steady_samples)),
        "max_abs_auxiliary_rad": float(np.max(np.abs(joint))),
        "yaw_delta_rad": float(unwrapped_yaw[-1] - unwrapped_yaw[0]),
        "max_abs_roll_rad": float(np.max(np.abs(roll))),
        "max_aux_speed_rad_s": float(np.max(np.abs(joint_velocity))),
        "max_height_deviation_m": float(np.max(np.abs(base_z - base_z[0]))),
        "max_abs_vertical_velocity_m_s": float(np.max(np.abs(vertical_velocity))),
        "rms_vertical_velocity_m_s": float(np.sqrt(np.mean(vertical_velocity**2))),
        "max_contact_force_norm": float(np.max(contact)),
        "collision": bool(np.any(collision_flags)),
        "samples": samples,
    }


def render_probe_plot(
    output_path: str | Path,
    records: Sequence[dict[str, object]],
    *,
    passed: bool,
) -> Path:
    """Render positive and negative steering traces to a standalone PNG."""

    from PIL import Image, ImageDraw

    output = Path(output_path)
    if not records:
        raise ValueError("at least one probe direction is required")
    parsed: list[tuple[str, np.ndarray]] = []
    for record in records:
        label = str(record.get("label", ""))
        samples = record.get("samples")
        if not isinstance(samples, list):
            raise ValueError("each direction must contain a samples list")
        points = _finite_trace(
            [sample["position_xy_m"] for sample in samples],
            name=f"{label} position_xy_m",
            columns=2,
        )
        parsed.append((label, points))

    width, height = 900, 650
    margin, header = 70, 90
    canvas = Image.new("RGB", (width, height), (249, 250, 252))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (margin, 24),
        f"Auxiliary steering probe: {'PASS' if passed else 'FAIL'}",
        fill=(25, 125, 70) if passed else (205, 45, 45),
    )
    draw.text((margin, 50), "world X right, world Y up; green=start, square=end", fill=(55, 60, 66))

    all_points = np.concatenate([points for _, points in parsed], axis=0)
    minimum = all_points.min(axis=0)
    maximum = all_points.max(axis=0)
    span = np.maximum(maximum - minimum, 0.25)
    center = 0.5 * (minimum + maximum)
    plot_width = width - 2 * margin
    plot_height = height - header - margin
    scale = min(plot_width / span[0], plot_height / span[1]) * 0.88

    def pixel(point: np.ndarray) -> tuple[int, int]:
        offset = point - center
        return (
            int(round(width / 2 + offset[0] * scale)),
            int(round(header + plot_height / 2 - offset[1] * scale)),
        )

    draw.rectangle(
        (margin, header, width - margin, height - margin),
        outline=(190, 196, 202),
        width=2,
    )
    colors = {"positive": (30, 105, 210), "negative": (225, 105, 35)}
    for record_index, (label, points) in enumerate(parsed):
        color = colors.get(label, (125, 75 + 30 * record_index, 160))
        pixels = [pixel(point) for point in points]
        if len(pixels) > 1:
            draw.line(pixels, fill=color, width=4)
        start_x, start_y = pixels[0]
        end_x, end_y = pixels[-1]
        draw.ellipse((start_x - 6, start_y - 6, start_x + 6, start_y + 6), fill=(35, 170, 80))
        draw.rectangle((end_x - 6, end_y - 6, end_x + 6, end_y + 6), fill=color)
        draw.text((margin + record_index * 190, height - 42), label, fill=color)

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    canvas.save(temporary, format="PNG")
    temporary.replace(output)
    return output


def _roll_yaw_from_wxyz(quaternion: object) -> tuple[float, float]:
    orientation = np.asarray(quaternion, dtype=np.float64)
    if orientation.shape != (4,) or not np.isfinite(orientation).all():
        raise RuntimeError("robot quaternion must be a finite vector with shape (4,)")
    norm = float(np.linalg.norm(orientation))
    if norm <= np.finfo(np.float64).eps:
        raise RuntimeError("robot quaternion must have non-zero norm")
    w, x, y, z = orientation / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, yaw


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")
    temporary.replace(path)


def run_probe(
    contract_path: Path,
    output_dir: Path,
    *,
    gui: bool,
    main_acceleration_rad_s2: float,
    auxiliary_kp: float,
    auxiliary_kd: float,
    auxiliary_rate_rad_s: float,
) -> dict[str, object]:
    """Run the ground-only Isaac probe. Isaac imports are intentionally local."""

    controller_parameters = np.asarray(
        [main_acceleration_rad_s2, auxiliary_kp, auxiliary_kd, auxiliary_rate_rad_s],
        dtype=np.float64,
    )
    if not np.isfinite(controller_parameters).all() or np.any(controller_parameters <= 0.0):
        raise ValueError(
            "main acceleration, auxiliary Kp, Kd, and rate limit must be positive and finite"
        )

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {"headless": not gui, "width": 1280, "height": 720}
    )
    try:
        from collection_schema import load_contract

        import omni.usd
        from isaacsim.core.prims import RigidPrim
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction
        from pxr import Usd, UsdPhysics

        contract = load_contract(contract_path)
        controls = probe_controls_from_contract(contract)

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / contract.schedule.physics_hz,
            rendering_dt=1.0 / contract.schedule.stereo_hz,
        )
        world.scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_ROOT)
        robot = world.scene.add(
            Articulation(prim_path=ROBOT_ROOT, name="probe_ball_robot", position=ROBOT_START)
        )

        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(ROBOT_ROOT)
        rigid_paths = [
            str(prim.GetPath())
            for prim in Usd.PrimRange(robot_prim)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        if not rigid_paths:
            raise RuntimeError("robot USD contains no rigid bodies for contact tracking")
        contacts = world.scene.add(
            RigidPrim(
                prim_paths_expr=rigid_paths,
                name="probe_robot_contacts",
                reset_xform_properties=False,
                track_contact_forces=True,
                contact_filter_prim_paths_expr=contact_filter_groups(rigid_paths),
            )
        )
        world.reset()
        if robot.num_dof < 2:
            raise RuntimeError("probe requires at least two robot joints")
        controller = robot.get_articulation_controller()
        controller.set_gains(
            kps=np.asarray([0.0, auxiliary_kp, 0.0]),
            kds=np.asarray([DEFAULT_MAIN_DERIVATIVE_GAIN, auxiliary_kd, 0.0]),
        )
        zero_action = ArticulationAction(
            joint_velocities=np.asarray([0.0, np.nan, np.nan]),
            joint_positions=np.asarray([np.nan, 0.0, np.nan]),
        )

        direction_records: list[dict[str, object]] = []
        for label, auxiliary_command in controls:
            robot.set_world_pose(
                position=ROBOT_START.copy(),
                orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
            )
            robot.set_linear_velocity(np.zeros(3))
            robot.set_angular_velocity(np.zeros(3))
            robot.set_joint_positions(np.zeros(robot.num_dof))
            robot.set_joint_velocities(np.zeros(robot.num_dof))
            controller.set_gains(
                kps=np.asarray([0.0, auxiliary_kp, 0.0]),
                kds=np.asarray([DEFAULT_MAIN_DERIVATIVE_GAIN, auxiliary_kd, 0.0]),
            )
            for _ in range(SETTLE_STEPS):
                controller.apply_action(zero_action)
                world.step(render=gui)

            current_main_target = 0.0
            current_auxiliary_target = 0.0
            main_target_trace: list[float] = []
            joint_target_trace: list[float] = []
            joint_trace: list[float] = []
            joint_velocity_trace: list[float] = []
            yaw_trace: list[float] = []
            roll_trace: list[float] = []
            base_z_trace: list[float] = []
            vertical_velocity_trace: list[float] = []
            contact_trace: list[float] = []
            collision_trace: list[bool] = []
            position_trace: list[list[float]] = []

            def record_sample(
                main_target_rad_s: float, joint_target_rad: float
            ) -> None:
                position, quaternion = robot.get_world_pose()
                roll, yaw = _roll_yaw_from_wxyz(quaternion)
                linear_velocity = np.asarray(robot.get_linear_velocity(), dtype=np.float64)
                joint_positions = np.asarray(robot.get_joint_positions(), dtype=np.float64)
                joint_velocities = np.asarray(robot.get_joint_velocities(), dtype=np.float64)
                forces = np.asarray(
                    contacts.get_net_contact_forces(dt=1.0 / contract.schedule.physics_hz),
                    dtype=np.float64,
                )
                if not np.isfinite(forces).all():
                    raise RuntimeError("contact force trace contains non-finite values")
                main_target_trace.append(float(main_target_rad_s))
                joint_target_trace.append(float(joint_target_rad))
                joint_trace.append(float(joint_positions[1]))
                joint_velocity_trace.append(float(joint_velocities[1]))
                yaw_trace.append(yaw)
                roll_trace.append(roll)
                base_z_trace.append(float(position[2]))
                vertical_velocity_trace.append(float(linear_velocity[2]))
                contact_trace.append(float(np.linalg.norm(forces, axis=-1).sum()))
                # The probe world contains only the expected ground contact.
                collision_trace.append(False)
                position_trace.append([float(position[0]), float(position[1])])

            record_sample(current_main_target, current_auxiliary_target)
            for _ in range(DRIVE_STEPS):
                if gui and not simulation_app.is_running():
                    raise RuntimeError("Isaac GUI was closed before the probe completed")
                current_main_target = slew_limited_target(
                    current_main_target,
                    MAIN_VELOCITY_RAD_S,
                    main_acceleration_rad_s2,
                    1.0 / contract.schedule.physics_hz,
                )
                current_auxiliary_target = slew_limited_target(
                    current_auxiliary_target,
                    auxiliary_command,
                    auxiliary_rate_rad_s,
                    1.0 / contract.schedule.physics_hz,
                )
                command = ArticulationAction(
                    joint_velocities=np.asarray([current_main_target, np.nan, np.nan]),
                    joint_positions=np.asarray(
                        [np.nan, current_auxiliary_target, np.nan]
                    ),
                )
                controller.apply_action(command)
                world.step(render=gui)
                record_sample(current_main_target, current_auxiliary_target)

            direction_records.append(
                summarize_direction(
                    label=label,
                    main_command_rad_s=MAIN_VELOCITY_RAD_S,
                    main_target_rad_s=main_target_trace,
                    auxiliary_command_rad=auxiliary_command,
                    joint_target_rad=joint_target_trace,
                    joint_rad=joint_trace,
                    yaw_rad=yaw_trace,
                    roll_rad=roll_trace,
                    joint_velocity_rad_s=joint_velocity_trace,
                    base_z_m=base_z_trace,
                    vertical_velocity_m_s=vertical_velocity_trace,
                    contact_force_norm=contact_trace,
                    collision=collision_trace,
                    position_xy=position_trace,
                )
            )

        evaluation = evaluate_probe(
            commanded_rad=[record["auxiliary_command_rad"] for record in direction_records],
            measured_rad=[record["measured_auxiliary_rad"] for record in direction_records],
            max_abs_measured_rad=[
                record["max_abs_auxiliary_rad"] for record in direction_records
            ],
            yaw_delta_rad=[record["yaw_delta_rad"] for record in direction_records],
            max_abs_roll_rad=[record["max_abs_roll_rad"] for record in direction_records],
            max_aux_speed_rad_s=[
                record["max_aux_speed_rad_s"] for record in direction_records
            ],
            max_height_deviation_m=[
                record["max_height_deviation_m"] for record in direction_records
            ],
            collision=[record["collision"] for record in direction_records],
        )
        report: dict[str, object] = {
            "schema_version": "1.3",
            "probe": "auxiliary_steering_limits",
            "action_protocol_id": contract.action_protocol_id,
            "passed": evaluation["passed"],
            "physics_hz": contract.schedule.physics_hz,
            "settle_steps": SETTLE_STEPS,
            "drive_steps": DRIVE_STEPS,
            "controller": {
                "main_acceleration_limit_rad_s2": main_acceleration_rad_s2,
                "auxiliary_kp": auxiliary_kp,
                "auxiliary_kd": auxiliary_kd,
                "auxiliary_target_rate_limit_rad_s": auxiliary_rate_rad_s,
            },
            "root_roll_semantics": (
                "diagnostic only: the rolling spherical base can have arbitrary roll"
            ),
            "contact_semantics": (
                "contact_force_norm records expected robot-ground contact; the ground-only "
                "probe contains no obstacle collision geometry"
            ),
            "evaluation": evaluation,
            "directions": direction_records,
        }
        output_dir.mkdir(parents=True, exist_ok=True)
        report_path = output_dir / "steering_probe.json"
        _atomic_json(report_path, report)
        _render_report_in_clean_process(report_path)
        return report
    finally:
        simulation_app.close()


def _render_report_file(report_path: Path) -> Path:
    with report_path.open(encoding="utf-8") as handle:
        report = json.load(handle)
    if not isinstance(report, dict) or not isinstance(report.get("directions"), list):
        raise ValueError("probe report must contain a directions list")
    if not isinstance(report.get("passed"), bool):
        raise ValueError("probe report must contain a boolean passed field")
    return render_probe_plot(
        report_path.with_name("steering_probe.png"),
        report["directions"],
        passed=report["passed"],
    )


def _render_report_in_clean_process(report_path: Path) -> Path:
    completed = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), "--render-report", str(report_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"clean probe plot renderer failed: {detail}")
    return report_path.with_name("steering_probe.png")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the low-speed two-direction auxiliary steering acceptance probe."
    )
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--gui", action="store_true", help="show the Isaac Sim window")
    parser.add_argument(
        "--main-accel-rad-s2",
        type=float,
        default=DEFAULT_MAIN_ACCEL_RAD_S2,
        help="maximum main-joint command acceleration in radians per second squared",
    )
    parser.add_argument("--aux-kp", type=float, default=DEFAULT_AUX_KP)
    parser.add_argument("--aux-kd", type=float, default=DEFAULT_AUX_KD)
    parser.add_argument(
        "--aux-rate-deg-s",
        type=float,
        default=DEFAULT_AUX_RATE_DEG_S,
        help="maximum auxiliary target slew rate in degrees per second",
    )
    parser.add_argument("--render-report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.render_report is None and args.out is None:
        parser.error("--out is required")
    if args.render_report is not None and (args.out is not None or args.gui):
        parser.error("--render-report cannot be combined with --out or --gui")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.render_report is not None:
        _render_report_file(args.render_report)
        return 0
    report = run_probe(
        args.contract,
        args.out,
        gui=args.gui,
        main_acceleration_rad_s2=args.main_accel_rad_s2,
        auxiliary_kp=args.aux_kp,
        auxiliary_kd=args.aux_kd,
        auxiliary_rate_rad_s=math.radians(args.aux_rate_deg_s),
    )
    print(f"Probe report: {args.out / 'steering_probe.json'}")
    print(f"Probe plot:   {args.out / 'steering_probe.png'}")
    print(f"Acceptance:   {'PASS' if report['passed'] else 'FAIL'}")
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
