"""LEGACY collector for the obsolete image/state dataset format.

Do not use this module for new PointGoal joint-world-model or BEV datasets.
Use ``collect_data_dubins.py`` and the schema 1.1 contract instead.

球型机器人数据采集脚本（含图像+位姿）
运行:
  python scripts/collect_data.py --headless \
    --scene /home/ubuntu/project/InternNav/data/scene_data/mp3d_pe/scans/29hnd4uzFmX/matterport_mesh/04eb2788768d40a38d35d876a02e9624/fixed.usd \
    --num_episodes 50 --episode_len 200

输出: data/trajectories/ep{N:04d}/
  left_{t:04d}.jpg     左目图像
  right_{t:04d}.jpg    右目图像
  states.npz           本体状态+动作+相机位姿序列
"""

from camera_rig import CAMERA_FORWARD_M, CAMERA_HALF_BASELINE_M, CAMERA_UP_M

import argparse, os, sys, json
import numpy as np
from pathlib import Path
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument("--headless",     action="store_true", default=True)
parser.add_argument("--scene",        type=str, default="")
parser.add_argument("--num_episodes", type=int, default=20)
parser.add_argument("--episode_len",  type=int, default=200)
parser.add_argument("--img_w",        type=int, default=320)
parser.add_argument("--img_h",        type=int, default=240)
parser.add_argument("--save_dir",     type=str, default="/home/an/project/ball_wm_project_full/research/ball_wm/data/trajectories")
parser.add_argument("--save_images",  action="store_true", default=True)
args = parser.parse_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.usd
from pxr import UsdGeom, Gf, UsdLux
import numpy as np
import carb
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera

ROBOT_USD = "/home/an/project/ball_wm_project_full/project/rl_env_0.4/rl_env/rotunbot_wheel_sim_model_0.4/Rotunbot_wheel.usd"
SAVE_DIR  = Path(args.save_dir)
SAVE_DIR.mkdir(parents=True, exist_ok=True)

world = World(stage_units_in_meters=1.0, physics_dt=1.0/50.0, rendering_dt=1.0/50.0)

ROBOT_START = np.array([0.0, 0.0, 0.45])

if args.scene and os.path.exists(args.scene):
    add_reference_to_stage(usd_path=os.path.abspath(args.scene), prim_path="/World/Scene")
    XFormPrim("/World/Scene").set_local_pose(translation=np.array([0.0, 0.0, 0.0]))
    print(f"场景: {args.scene}")
else:
    world.scene.add_default_ground_plane()

stage = omni.usd.get_context().get_stage()

def _add_lights():
    for p in ["/World/SunLight", "/World/up_disk", "/World/dn_disk"]:
        stage.RemovePrim(p)
    dist = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    dist.CreateIntensityAttr(1000.0)
    for path, z_off, rot in [("/World/up_disk", 1.0, 180.0), ("/World/dn_disk", -1.0, 0.0)]:
        dk = UsdLux.DiskLight.Define(stage, path)
        dk.CreateIntensityAttr(5000.0)
        dk.CreateRadiusAttr(50.0)
        xf = UsdGeom.Xformable(dk)
        xf.ClearXformOpOrder()
        xf.AddRotateXYZOp().Set(Gf.Vec3f(rot, 0.0, 0.0))
        xf.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, z_off))
    carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLighting")

_add_lights()

add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/BallRobot")
robot = world.scene.add(
    Articulation(prim_path="/World/BallRobot", name="ball_robot", position=ROBOT_START)
)

CAM_X, CAM_Y, CAM_Z = CAMERA_FORWARD_M, CAMERA_HALF_BASELINE_M, CAMERA_UP_M
FOCAL_LEN, H_APT = 16.45, 18.0
V_APT = H_APT * args.img_h / args.img_w

def _make_cam_usd(path, y_offset):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.GetFocalLengthAttr().Set(FOCAL_LEN)
    cam.GetHorizontalApertureAttr().Set(H_APT)
    cam.GetVerticalApertureAttr().Set(V_APT)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(CAM_X, y_offset, CAM_Z))
    xf.AddRotateYZXOp().Set(Gf.Vec3f(0.0, -90.0, 0.0))

_make_cam_usd("/World/BallRobot/base_link/cam_left",  CAM_Y)
_make_cam_usd("/World/BallRobot/base_link/cam_right", -CAM_Y)

world.reset()

kps = np.array([0.0, 500.0, 0.0])
kds = np.array([80.0,  50.0, 0.0])
robot.get_articulation_controller().set_gains(kps=kps, kds=kds)

cam_left  = Camera(prim_path="/World/BallRobot/base_link/cam_left",
                   resolution=(args.img_w, args.img_h), frequency=10)
cam_right = Camera(prim_path="/World/BallRobot/base_link/cam_right",
                   resolution=(args.img_w, args.img_h), frequency=10)
cam_left.initialize()
cam_right.initialize()

print(f"\n关节: {robot.dof_names}")
print(f"图像: {args.img_w}×{args.img_h}  保存图像: {args.save_images}")
print(f"Episodes: {args.num_episodes}×{args.episode_len}步\n")


MOTION_PATTERNS = [
    ("straight",  lambda t: (1.0,  0.0)),
    ("left_turn", lambda t: (1.0, -0.2)),
    ("right_turn",lambda t: (1.0,  0.2)),
    ("s_curve",   lambda t: (1.0,  0.2 * np.sin(t * 0.1))),
    ("sway",      lambda t: (0.0,  0.2 * np.sin(t * 0.15))),
]


def reset_robot(ep: int):
    np.random.seed(ep)
    x = np.random.uniform(-1.0, 1.0)
    y = np.random.uniform(-1.0, 1.0)
    robot.set_world_pose(position=np.array([x, y, ROBOT_START[2]]))
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    robot.set_joint_positions(np.zeros(robot.num_dof))
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    robot.get_articulation_controller().set_gains(kps=kps, kds=kds)
    world.step(render=False)


def get_cam_pose(cam_prim_path: str):
    prim = stage.GetPrimAtPath(cam_prim_path)
    xf_cache = UsdGeom.XformCache()
    mat = xf_cache.GetLocalToWorldTransform(prim)
    pose = np.array(mat).reshape(4, 4).T
    return pose.astype(np.float32)


def get_frame(cam: Camera):
    frame = cam.get_current_frame()
    if frame and "rgba" in frame and frame["rgba"] is not None:
        rgba = frame["rgba"]
        if rgba is not None and len(rgba.shape) == 3:
            return rgba[:, :, :3]
    return None


total_frames = 0

for ep in range(args.num_episodes):
    reset_robot(ep)

    pattern_name, vel_fn = MOTION_PATTERNS[ep % len(MOTION_PATTERNS)]
    ep_dir = SAVE_DIR / f"ep{ep:04d}"
    if args.save_images:
        ep_dir.mkdir(exist_ok=True)

    buf = {
        "pos":       [],
        "quat":      [],
        "lin_vel":   [],
        "ang_vel":   [],
        "joint_pos": [],
        "joint_vel": [],
        "action":    [],
        "cam_left_pose":  [],
        "cam_right_pose": [],
    }

    for step in range(args.episode_len):
        v_target, turn_target = vel_fn(step)

        world.step(render=args.save_images)

        pos, quat = robot.get_world_pose()
        buf["pos"].append(pos.copy())
        buf["quat"].append(quat.copy())
        buf["lin_vel"].append(robot.get_linear_velocity().copy())
        buf["ang_vel"].append(robot.get_angular_velocity().copy())
        buf["joint_pos"].append(robot.get_joint_positions().copy())
        buf["joint_vel"].append(robot.get_joint_velocities().copy())
        buf["action"].append(np.array([v_target, turn_target, 0.0], dtype=np.float32))
        buf["cam_left_pose"].append(get_cam_pose("/World/BallRobot/base_link/cam_left"))
        buf["cam_right_pose"].append(get_cam_pose("/World/BallRobot/base_link/cam_right"))

        robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_velocities=np.array([v_target, np.nan, np.nan]),
                joint_positions=np.array([np.nan, turn_target, np.nan]),
            )
        )

        if args.save_images and step % 2 == 0:
            lf = get_frame(cam_left)
            rf = get_frame(cam_right)
            if lf is not None:
                Image.fromarray(lf.astype(np.uint8)).save(ep_dir / f"left_{step:04d}.jpg")
            if rf is not None:
                Image.fromarray(rf.astype(np.uint8)).save(ep_dir / f"right_{step:04d}.jpg")

    states = {k: np.stack(v, axis=0) for k, v in buf.items()}
    states["pattern"] = np.array([pattern_name])
    np.savez_compressed(ep_dir / "states.npz" if args.save_images else
                        SAVE_DIR / f"ep{ep:04d}.npz", **states)

    total_frames += args.episode_len
    print(f"  [{pattern_name:10s}] ep{ep+1:3d}/{args.num_episodes}  "
          f"帧数:{args.episode_len}  累计:{total_frames}")

print(f"\n采集完成！总帧数: {total_frames}")
print(f"保存目录: {SAVE_DIR}")
simulation_app.close()
