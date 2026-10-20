"""Isolated fixed-camera indoor lighting/roof comparison; truth is offline-only."""

import json
import os
import sys
from pathlib import Path

import numpy as np

import scripts.run_pointgoal_benchmark as runner


HERE = Path(__file__).resolve().parent
ORACLE_EXPERIMENT = HERE.parent / "oracle_feasible_route_20260929_v9" / "gated_speed_012"
sys.path.insert(0, str(ORACLE_EXPERIMENT))
from roll_pid import install_runtime  # noqa: E402

args = runner._build_parser().parse_args()
room_variant = os.environ.get("ROOM_VARIANT", "lamps")
if room_variant not in {"lamps", "roof_lamps", "sun_off", "hide_base_visual", "probe_card"}:
    raise ValueError("invalid ROOM_VARIANT")
capture_stride = int(os.environ.get("STEREO_CAPTURE_STRIDE", "0"))
if capture_stride < 0:
    raise ValueError("STEREO_CAPTURE_STRIDE must be non-negative")
config = json.loads((ORACLE_EXPERIMENT / "selected_candidate.json").read_text())
base = install_runtime(
    runner.IsaacBenchmarkRuntime,
    **config["gains"],
    output=args.run_root / "feedback",
)


class DepthAuditRuntime(base):
    SELECTED = {0, 10, 30, 50, 70, 90, 110, 130}

    def __init__(self, *init_args, **init_kwargs):
        super().__init__(*init_args, **init_kwargs)
        self._audit_capture_index = 0

    def _initialize(self, scene_path, occupancy_path, episode):
        super()._initialize(scene_path, occupancy_path, episode)
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdLux

        # Camera prims, robot, scene and controller remain exactly as loaded.
        # Lights and the optional non-colliding roof exist only in this runtime.
        stage = omni.usd.get_context().get_stage()
        if room_variant == "probe_card":
            card = UsdGeom.Cube.Define(stage, "/World/DiagnosticProbeCard")
            card.CreateSizeAttr(0.6)
            card.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.1, 0.1)])
            xform = UsdGeom.Xformable(card)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(-1.9, 0.0, 0.6))
        elif room_variant == "hide_base_visual":
            base_mesh = stage.GetPrimAtPath(
                "/World/BallRobot/base_link/visuals/base_link/mesh")
            if not base_mesh.IsValid():
                raise RuntimeError("base_link visual mesh missing")
            UsdGeom.Imageable(base_mesh).GetVisibilityAttr().Set("invisible")
        elif room_variant == "sun_off":
            stage.RemovePrim("/World/SunLight")
        else:
            for index, x in enumerate((-3.0, 0.0, 3.0)):
                light = UsdLux.SphereLight.Define(stage, f"/World/DiagnosticRoomLight_{index}")
                light.CreateIntensityAttr(10000.0)
                light.CreateRadiusAttr(0.5)
                xform = UsdGeom.Xformable(light)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 2.2))
        if room_variant == "roof_lamps":
            roof = UsdGeom.Cube.Define(stage, "/World/DiagnosticRoomRoof")
            roof.CreateSizeAttr(1.0)
            xform = UsdGeom.Xformable(roof)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 3.2))
            xform.AddScaleOp().Set(Gf.Vec3f(11.0, 10.0, 0.2))
        self._world.render()
        audit = {
            "room_variant": room_variant,
            "lights": [{
                "path": f"/World/DiagnosticRoomLight_{i}",
                "valid": bool(stage.GetPrimAtPath(f"/World/DiagnosticRoomLight_{i}").IsValid()),
                "intensity": float(UsdLux.SphereLight(stage.GetPrimAtPath(
                    f"/World/DiagnosticRoomLight_{i}")).GetIntensityAttr().Get()),
            } for i in range(3)] if room_variant in {"lamps", "roof_lamps"} else [],
            "roof_valid": bool(stage.GetPrimAtPath("/World/DiagnosticRoomRoof").IsValid()),
            "probe_card_valid": bool(stage.GetPrimAtPath("/World/DiagnosticProbeCard").IsValid()),
            "sun_valid": bool(stage.GetPrimAtPath("/World/SunLight").IsValid()),
            "base_visual_visibility": UsdGeom.Imageable(stage.GetPrimAtPath(
                "/World/BallRobot/base_link/visuals/base_link/mesh"))
                .GetVisibilityAttr().Get(),
            "roof_world_transform": (
                [[float(v) for v in row] for row in
                 UsdGeom.Xformable(stage.GetPrimAtPath("/World/DiagnosticRoomRoof"))
                 .ComputeLocalToWorldTransform(Usd.TimeCode.Default())]
                if room_variant == "roof_lamps" else None
            ),
        }
        (args.run_root / "lighting_stage_audit.json").write_text(json.dumps(audit, indent=2) + "\n")

    def _capture(self):
        step = super()._capture()
        index = self._audit_capture_index
        self._audit_capture_index += 1
        if index == 0:
            import omni.usd
            stage_now = omni.usd.get_context().get_stage()
            (args.run_root / "stage_at_first_capture.json").write_text(json.dumps({
                "probe_card_valid": bool(stage_now.GetPrimAtPath(
                    "/World/DiagnosticProbeCard").IsValid()),
                "roof_valid": bool(stage_now.GetPrimAtPath(
                    "/World/DiagnosticRoomRoof").IsValid()),
                "light_0_valid": bool(stage_now.GetPrimAtPath(
                    "/World/DiagnosticRoomLight_0").IsValid()),
                "sun_valid": bool(stage_now.GetPrimAtPath("/World/SunLight").IsValid()),
            }, indent=2) + "\n")
        if index in self.SELECTED or (capture_stride > 0 and index % capture_stride == 0):
            from bev_ground_truth import read_camera_depth
            from collect_data_dubins import LEFT_CAMERA_PATH, RIGHT_CAMERA_PATH, get_cam_pose
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            left_depth = read_camera_depth(self._left_camera)
            right_depth = read_camera_depth(self._right_camera)
            left_rgb = step.observation.stereo_left_rgb
            right_rgb = step.observation.stereo_right_rgb
            if left_depth.shape != left_rgb.shape[:2] or right_depth.shape != right_rgb.shape[:2]:
                raise RuntimeError("renderer depth/RGB shapes differ")
            position, quaternion = self._robot.get_world_pose()
            output = args.run_root / "depth_truth"
            output.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(
                output / f"frame_{index:03d}.npz",
                left_rgb=left_rgb,
                right_rgb=right_rgb,
                left_depth_m=left_depth,
                right_depth_m=right_depth,
                left_camera_usd_world=get_cam_pose(stage, LEFT_CAMERA_PATH),
                right_camera_usd_world=get_cam_pose(stage, RIGHT_CAMERA_PATH),
                root_position_world_m=np.asarray(position),
                root_quaternion_world_wxyz=np.asarray(quaternion),
                simulation_time_s=np.asarray(step.simulation_time_s),
            )
        return step


runner.IsaacBenchmarkRuntime = DepthAuditRuntime
original_child = runner._child_command


def child(*child_args, **child_kwargs):
    command = original_child(*child_args, **child_kwargs)
    command[1] = str(HERE / "room_light_capture_worker.py")
    return command


runner._child_command = child
args.run_root.mkdir(parents=True, exist_ok=True)
(args.run_root / "depth_truth_usage.json").write_text(
    json.dumps({
        "policy": "goal_greedy",
        "truth_role": "offline perception diagnosis only; runtime capture side-channel",
        "sample_capture_indices": sorted(DepthAuditRuntime.SELECTED),
        "capture_stride": capture_stride,
        "roll_pid_gains": config["gains"],
        "rig_variant": "frozen",
        "rig_positions_base_link_m": [[0.1, 0.3, 0.1], [0.1, -0.3, 0.1]],
        "lighting_variant": room_variant,
        "lighting_configuration": "three SphereLights at x=-3,0,3; y=0; z=2.2 m; intensity=10000; radius=0.5 m",
        "roof_configuration": "visual-only cube 11x10x0.2 m centered at z=3.2 m" if room_variant == "roof_lamps" else None,
    }, indent=2) + "\n"
)
raise SystemExit(runner.main())
