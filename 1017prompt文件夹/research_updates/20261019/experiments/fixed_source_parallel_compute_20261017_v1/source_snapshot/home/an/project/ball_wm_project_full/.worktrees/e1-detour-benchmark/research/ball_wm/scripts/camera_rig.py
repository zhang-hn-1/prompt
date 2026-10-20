"""Current stereo installation, in metres in the base_link (ball-centre) frame.

+X is forward, +Y is left, +Z is up. These are local offsets: cameras
remain attached to base_link and move with it. Historical rigs/data keep
their recorded extrinsics; never relabel them with this installation.
"""
CAMERA_RIG_ID = "ball_stereo_front020_up005_v2"
CAMERA_FORWARD_M = 0.20
CAMERA_HALF_BASELINE_M = 0.30
CAMERA_UP_M = 0.05
CAMERA_ROTATE_YZX_DEG = (0.0, -90.0, 0.0)


def camera_translation_base_link(y_offset):
    return (CAMERA_FORWARD_M, float(y_offset), CAMERA_UP_M)


def camera_rig_metadata():
    return {
        "rig_id": CAMERA_RIG_ID,
        "parent_frame": "base_link",
        "units": "m",
        "left_position_base_link_m": list(camera_translation_base_link(CAMERA_HALF_BASELINE_M)),
        "right_position_base_link_m": list(camera_translation_base_link(-CAMERA_HALF_BASELINE_M)),
        "rotate_yzx_deg": list(CAMERA_ROTATE_YZX_DEG),
    }
