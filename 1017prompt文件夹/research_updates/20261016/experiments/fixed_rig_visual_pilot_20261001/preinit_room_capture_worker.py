"""Author diagnostic geometry before Isaac camera initialization."""

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
room_variant = os.environ.get("ROOM_VARIANT", "probe_card")
if room_variant not in {"probe_card", "lamps", "roof_lamps"}:
    raise ValueError("ROOM_VARIANT must be probe_card, lamps or roof_lamps")
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
        # Isaac's camera annotators did not reflect prims added after the
        # original _initialize completed. Wrap the scene-load boundary so
        # diagnostic content exists before camera/render-product creation.
        scripts_dir = Path(runner.__file__).resolve().parent
        if str(scripts_dir) not in sys.path:
            sys.path.insert(0, str(scripts_dir))
        import collect_data_dubins

        original_loader = collect_data_dubins._load_collection_scene

        def loader(*loader_args, **loader_kwargs):
            scene_root = original_loader(*loader_args, **loader_kwargs)
            import omni.usd
            from pxr import Gf, UsdGeom, UsdLux

            stage = omni.usd.get_context().get_stage()
            if room_variant == "probe_card":
                card = UsdGeom.Cube.Define(stage, "/World/DiagnosticProbeCard")
                card.CreateSizeAttr(0.6)
                card.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.1, 0.1)])
                transform = UsdGeom.Xformable(card)
                transform.ClearXformOpOrder()
                transform.AddTranslateOp().Set(Gf.Vec3d(-1.9, 0.0, 0.6))
            else:
                for index, x in enumerate((-3.0, 0.0, 3.0)):
                    light = UsdLux.SphereLight.Define(
                        stage, f"/World/DiagnosticRoomLight_{index}")
                    light.CreateIntensityAttr(10000.0)
                    light.CreateRadiusAttr(0.5)
                    transform = UsdGeom.Xformable(light)
                    transform.ClearXformOpOrder()
                    transform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 2.2))
                if room_variant == "roof_lamps":
                    roof = UsdGeom.Cube.Define(stage, "/World/DiagnosticRoomRoof")
                    roof.CreateSizeAttr(1.0)
                    transform = UsdGeom.Xformable(roof)
                    transform.ClearXformOpOrder()
                    transform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 3.2))
                    transform.AddScaleOp().Set(Gf.Vec3f(11.0, 10.0, 0.2))
            return scene_root

        collect_data_dubins._load_collection_scene = loader
        try:
            super()._initialize(scene_path, occupancy_path, episode)
        finally:
            collect_data_dubins._load_collection_scene = original_loader
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        (args.run_root / "preinit_stage_audit.json").write_text(json.dumps({
            "variant": room_variant,
            "probe_card_valid": bool(stage.GetPrimAtPath(
                "/World/DiagnosticProbeCard").IsValid()),
            "roof_valid": bool(stage.GetPrimAtPath(
                "/World/DiagnosticRoomRoof").IsValid()),
            "light_0_valid": bool(stage.GetPrimAtPath(
                "/World/DiagnosticRoomLight_0").IsValid()),
        }, indent=2) + "\n")

    def _capture(self):
        step = super()._capture()
        index = self._audit_capture_index
        self._audit_capture_index += 1
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
    command[1] = str(HERE / "preinit_room_capture_worker.py")
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
        "stage_authoring": "before camera initialization via scene-load wrapper",
    }, indent=2) + "\n"
)
raise SystemExit(runner.main())
