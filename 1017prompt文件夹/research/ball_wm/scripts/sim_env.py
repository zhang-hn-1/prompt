"""
球型机器人仿真环境 — 键盘交互控制
运行: python scripts/sim_env.py [--headless]

键盘控制 (需要终端焦点):
  W / ↑  : 前进
  S / ↓  : 后退
  A / ←  : 左转
  D / →  : 右转
  Space  : 停止
  R      : 重置位置
  Q/ESC  : 退出

关节:
  joint1 - 前进滚动 (effort_limit=160N)
  joint2 - 转向重锤摆 (effort_limit=200N)
  joint3 - 辅助关节 (effort_limit=18N, damping=5)
"""

import argparse
import sys
import threading
import select
import tty
import termios

parser = argparse.ArgumentParser()
parser.add_argument("--headless",    action="store_true")
parser.add_argument("--num_steps",  type=int,   default=500)
parser.add_argument("--record",     action="store_true", help="录制双目摄像头视频到 data/videos/")
parser.add_argument("--video_fps",  type=int,   default=25, help="视频帧率")
parser.add_argument("--scene",     type=str,   default="",    help="场景USD路径，为空则只用默认地面")
parser.add_argument("--robot_z",   type=float, default=0.45,  help="机器人初始高度 (m)")
parser.add_argument("--robot_x",   type=float, default=0.0,   help="机器人初始X位置 (m)")
parser.add_argument("--robot_y",   type=float, default=0.0,   help="机器人初始Y位置 (m)")
parser.add_argument("--cam_x",     type=float, default=0.1,   help="摄像头X偏移(前后, m)")
parser.add_argument("--cam_y",     type=float, default=0.30,  help="摄像头Y偏移(左右半距, m)")
parser.add_argument("--cam_z",     type=float, default=0.10,  help="摄像头Z偏移(高于球心, m)")
parser.add_argument("--fwd_vel",   type=float, default=1.0,   help="前进目标速度 (rad/s, joint1)")
parser.add_argument("--turn_pos",  type=float, default=0.2,   help="转向摆角目标 (rad, ~11度)")
parser.add_argument("--kp_vel",    type=float, default=80.0,  help="joint1速度控制增益(damping)")
parser.add_argument("--kp_pos",    type=float, default=500.0, help="joint2位置控制增益(stiffness)")
parser.add_argument("--kd_pos",        type=float, default=50.0,  help="joint2位置控制阻尼(damping)")
parser.add_argument("--use_scene_origin", action="store_true", help="机器人位置相对场景坐标系原点（用于Wanderland等场景）")
args = parser.parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "hide_ui": True,
    "width": 1280,
    "height": 720,
    "active_gpu": 0,
    "physics_gpu": 0,
    "multi_gpu": False,
    "anti_aliasing": 0,
    "renderer": "RaytracedLighting",
    "create_new_stage": False,
})

import numpy as np
import omni.usd
from pxr import UsdGeom, Gf
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.sensor import Camera

import os
from omni.isaac.core.prims import XFormPrim

ROBOT_USD = "/home/an/project/ball_wm_project_full/project/rl_env_0.4/rl_env/rotunbot_wheel_sim_model_0.4/Rotunbot_wheel.usd"
INIT_POS  = np.array([args.robot_x, args.robot_y, args.robot_z])

create_new_stage()
world = World(stage_units_in_meters=1.0, physics_dt=1.0/50.0, rendering_dt=1.0/50.0)

SCENE_CENTER = np.array([0.0, 0.0, 0.0])
SCENE_FLOOR_Z = 0.0

if args.scene and os.path.exists(args.scene):
    scene_path = os.path.abspath(args.scene)
    add_reference_to_stage(usd_path=scene_path, prim_path="/World/Scene")
    XFormPrim("/World/Scene").set_local_pose(translation=np.array([0.0, 0.0, 0.0]))



    from pxr import UsdGeom as _UsdGeom
    import omni.usd as _ousd2
    _stage = _ousd2.get_context().get_stage()
    _bbox = _UsdGeom.BBoxCache(0, ["default", "render"])
    _root = _stage.GetPrimAtPath("/World/Scene")
    if _root.IsValid():
        _b = _bbox.ComputeWorldBound(_root).GetRange()
        _mn, _mx = _b.GetMin(), _b.GetMax()
        SCENE_CENTER = np.array([(_mn[0]+_mx[0])/2, (_mn[1]+_mx[1])/2, (_mn[2]+_mx[2])/2])
        SCENE_FLOOR_Z = float(_mn[2])
        print(f"  场景已加载: {scene_path}")
        print(f"  场景范围: ({_mn[0]:.1f},{_mn[1]:.1f},{_mn[2]:.1f}) ~ ({_mx[0]:.1f},{_mx[1]:.1f},{_mx[2]:.1f})")
        print(f"  地面Z={SCENE_FLOOR_Z:.2f}  中心=({SCENE_CENTER[0]:.1f},{SCENE_CENTER[1]:.1f})")
else:
    world.scene.add_default_ground_plane()
    if args.scene:
        print(f"  警告: 场景文件不存在 {args.scene}，使用默认地面")

from pxr import UsdLux
import omni.usd as _ousd
import carb

def _add_lights(cx=0.0, cy=0.0, cz=0.0):
    _stage = _ousd.get_context().get_stage()

    for p in ["/World/DomeLight", "/World/SunLight",
              "/World/up_disk_light", "/World/down_disk_light"]:
        _stage.RemovePrim(p)

    distant = UsdLux.DistantLight.Define(_stage, "/World/SunLight")
    distant.CreateIntensityAttr(1000.0)
    distant.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    up = UsdLux.DiskLight.Define(_stage, "/World/up_disk_light")
    up.CreateIntensityAttr(5000.0)
    up.CreateRadiusAttr(50.0)
    up.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    xf_up = UsdGeom.Xformable(up)
    xf_up.ClearXformOpOrder()
    xf_up.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
    xf_up.AddTranslateOp().Set(Gf.Vec3f(cx, cy, cz + 1.0))

    dn = UsdLux.DiskLight.Define(_stage, "/World/down_disk_light")
    dn.CreateIntensityAttr(5000.0)
    dn.CreateRadiusAttr(50.0)
    dn.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    xf_dn = UsdGeom.Xformable(dn)
    xf_dn.ClearXformOpOrder()
    xf_dn.AddTranslateOp().Set(Gf.Vec3f(cx, cy, cz - 1.0))

    settings = carb.settings.get_settings()
    settings.set("/rtx/rendermode", "RaytracedLighting")

if args.scene and os.path.exists(args.scene) and args.use_scene_origin:
    INIT_POS = np.array([
        SCENE_CENTER[0] + args.robot_x,
        SCENE_CENTER[1] + args.robot_y,
        SCENE_FLOOR_Z + args.robot_z,
    ])
    print(f"  机器人初始位置(场景坐标系): ({INIT_POS[0]:.2f},{INIT_POS[1]:.2f},{INIT_POS[2]:.2f})")

_add_lights(cx=float(INIT_POS[0]), cy=float(INIT_POS[1]), cz=float(INIT_POS[2]))

add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/BallRobot")
robot = world.scene.add(
    Articulation(prim_path="/World/BallRobot", name="ball_robot", position=INIT_POS)
)
world.reset()

from omni.isaac.core.utils.viewports import set_camera_view
set_camera_view(
    eye=np.array([INIT_POS[0] - 5.0, INIT_POS[1], INIT_POS[2] + 4.0]),
    target=INIT_POS.copy(),
)

# joint1: stiffness=0(速度控制不需要位置刚度), damping=kp_vel(速度增益)
# joint2: stiffness=kp_pos(位置刚度), damping=kd_pos(阻尼)
# joint3: 全零，不参与主动控制
kps = np.array([0.0,          args.kp_pos, 0.0])
kds = np.array([args.kp_vel,  args.kd_pos, 0.0])
robot.get_articulation_controller().set_gains(kps=kps, kds=kds)

# 双目摄像头
# 挂在 base_link 上，随球体运动
# 位置相对于球心：左/右 ±0.3m，高度 +0.1m，朝前 (X轴)
# 使用 USD API 直接创建（跟随 base_link 坐标系）
stage = omni.usd.get_context().get_stage()

CAM_W, CAM_H = 640, 480
FOCAL_LEN     = 16.45   # mm
H_APERTURE    = 18.0    # mm  → hfov ≈ 79°
V_APERTURE    = H_APERTURE * CAM_H / CAM_W

def _make_camera(prim_path, x, y, z):
    cam = UsdGeom.Camera.Define(stage, prim_path)
    cam.GetFocalLengthAttr().Set(FOCAL_LEN)
    cam.GetHorizontalApertureAttr().Set(H_APERTURE)
    cam.GetVerticalApertureAttr().Set(V_APERTURE)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    # USD Camera默认朝 -Z 轴，旋转使其朝 +X 方向
    # 绕Y轴 +90° → 朝 +X；再绕Z轴调整左右摄像头使其水平
    xf.AddRotateYZXOp().Set(Gf.Vec3f(0.0, -90.0, 0.0))
    return cam

CAM_L_POS = (args.cam_x, args.cam_y,  args.cam_z)
CAM_R_POS = (args.cam_x, -args.cam_y, args.cam_z)

_make_camera("/World/BallRobot/base_link/cam_left",  *CAM_L_POS)
_make_camera("/World/BallRobot/base_link/cam_right", *CAM_R_POS)

cam_left  = Camera(prim_path="/World/BallRobot/base_link/cam_left",
                   resolution=(CAM_W, CAM_H), frequency=10)
cam_right = Camera(prim_path="/World/BallRobot/base_link/cam_right",
                   resolution=(CAM_W, CAM_H), frequency=10)
cam_left.initialize()
cam_right.initialize()

import cv2, datetime
_video_left = _video_right = None
if args.record:
    import os as _os
    _vid_dir = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), "data", "videos")
    _os.makedirs(_vid_dir, exist_ok=True)
    _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    _fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    _video_left  = cv2.VideoWriter(_os.path.join(_vid_dir, f"{_ts}_left.mp4"),  _fourcc, args.video_fps, (CAM_W, CAM_H))
    _video_right = cv2.VideoWriter(_os.path.join(_vid_dir, f"{_ts}_right.mp4"), _fourcc, args.video_fps, (CAM_W, CAM_H))
    print(f"  录制中 → {_vid_dir}/{_ts}_{{left,right}}.mp4")

print("\n" + "="*55)
print("  球型机器人仿真 — 键盘控制")
print("="*55)
print(f"  关节数: {robot.num_dof}  名称: {robot.dof_names}")
print(f"  摄像头: 左({args.cam_x:+.2f},{+args.cam_y:+.2f},{args.cam_z:+.2f})  "
      f"右({args.cam_x:+.2f},{-args.cam_y:+.2f},{args.cam_z:+.2f})  {CAM_W}×{CAM_H}@10Hz")
print(f"  前进速度: {args.fwd_vel} rad/s   转向摆角: ±{np.degrees(args.turn_pos):.1f}°")
print(f"  joint1 damping={args.kp_vel}   joint2 stiffness={args.kp_pos} damping={args.kd_pos}")
print("-"*55)
print("  W/↑  前进    S/↓  后退")
print("  A/←  左转    D/→  右转")
print("  Space 停止   R 重置   Q/ESC 退出")
print("="*55 + "\n")

_key = ""
_key_lock = threading.Lock()
_running = True
_need_reset = False


def _read_keys():
    global _key, _running, _need_reset
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while _running:
            if select.select([sys.stdin], [], [], 0.02)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    rest = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.05)[0] else ""
                    seq = ch + rest
                    with _key_lock:
                        if   seq == "\x1b[A": _key = "w"
                        elif seq == "\x1b[B": _key = "s"
                        elif seq == "\x1b[C": _key = "d"
                        elif seq == "\x1b[D": _key = "a"
                        else:                 _key = "q"
                else:
                    with _key_lock:
                        _key = ch.lower()
                    if ch.lower() == "r":
                        _need_reset = True
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


key_thread = threading.Thread(target=_read_keys, daemon=True)
key_thread.start()

FWD_VEL  = args.fwd_vel
TURN_POS = args.turn_pos
step = 0

while simulation_app.is_running():
    with _key_lock:
        k = _key

    if _need_reset:
        _need_reset = False
        robot.set_world_pose(position=INIT_POS)
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
        robot.set_joint_positions(np.zeros(robot.num_dof))
        robot.set_joint_velocities(np.zeros(robot.num_dof))
        robot.get_articulation_controller().set_gains(kps=kps, kds=kds)
        world.step(render=args.record or not args.headless)
        print("  [R] 已重置")
        step = 0
        continue

    if k in ("q", "\x1b", "\x03"):
        print("\n  退出")
        _running = False
        break

    vel_target  = 0.0
    turn_target = 0.0
    if   k == "w": vel_target =  FWD_VEL
    elif k == "s": vel_target = -FWD_VEL
    elif k == "a": vel_target =  FWD_VEL;  turn_target = -TURN_POS
    elif k == "d": vel_target =  FWD_VEL;  turn_target =  TURN_POS

    robot.get_articulation_controller().apply_action(
        ArticulationAction(
            joint_velocities=np.array([vel_target, np.nan, np.nan]),
            joint_positions=np.array([np.nan, turn_target, np.nan]),
        )
    )
    world.step(render=args.record or not args.headless)

    if args.record and _video_left is not None:
        def _grab(cam):
            f = cam.get_current_frame()
            if f and "rgba" in f and f["rgba"] is not None:
                rgba = f["rgba"]
                if rgba is not None and rgba.ndim == 3:
                    rgb = rgba[:, :, :3]
                    if rgb.dtype != np.uint8:
                        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
                    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return None
        lf = _grab(cam_left)
        rf = _grab(cam_right)
        if lf is not None: _video_left.write(lf)
        if rf is not None: _video_right.write(rf)

    if step % 50 == 0:
        pos = robot.get_world_pose()[0]
        lin = robot.get_linear_velocity()
        jv  = robot.get_joint_velocities()
        jp  = robot.get_joint_positions()
        key_disp = {"w":"前进","s":"后退","a":"左转","d":"右转","":"停止"}.get(k, k)
        print(
            f"  [{key_disp:2s}] step={step:4d} | "
            f"pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f}) | "
            f"linvel=({lin[0]:+.2f},{lin[1]:+.2f}) | "
            f"j1vel={jv[0]:+.2f}(tgt={vel_target:+.1f}) "
            f"j2pos={jp[1]:+.3f}(tgt={turn_target:+.2f})"
        )

    step += 1
    if args.headless and step >= args.num_steps:
        break

_running = False
if _video_left is not None:
    _video_left.release()
    _video_right.release()
    print("  视频已保存")
simulation_app.close()
