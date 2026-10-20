"""Offline fixed-rig observability: motion stereo and planar-image evidence.

Renderer depth is used for labels and scoring only. All candidates use the
unchanged base_link-mounted camera poses recorded with the RGB frames.
"""

import json
from pathlib import Path

import cv2
import numpy as np

from vertical_depth import FOCAL_LENGTH_PX


ROOT = Path(__file__).resolve().parent
CX, CY = 319.5, 239.5


def binary_auc(labels: np.ndarray, scores: np.ndarray) -> float | None:
    positive = np.asarray(scores)[np.asarray(labels, dtype=bool)]
    negative = np.sort(np.asarray(scores)[~np.asarray(labels, dtype=bool)])
    if not len(positive) or not len(negative):
        return None
    lower = np.searchsorted(negative, positive, side="left")
    upper = np.searchsorted(negative, positive, side="right")
    return float(np.sum((lower + upper) / 2.0) / (len(positive) * len(negative)))


def camera_geometry(transform: np.ndarray):
    # USD camera axes: +X=image right, -Y=image down, -Z=optical forward.
    pose = np.asarray(transform, dtype=np.float64)
    rotation = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
    return rotation, pose[:3, 3]


def project_world(points: np.ndarray, rotation: np.ndarray, center: np.ndarray):
    camera = (points - center) @ rotation
    with np.errstate(divide="ignore", invalid="ignore"):
        u = CX + FOCAL_LENGTH_PX * camera[..., 0] / camera[..., 2]
        v = CY + FOCAL_LENGTH_PX * camera[..., 1] / camera[..., 2]
    return u, v, camera[..., 2]


def paired_frames(scene: str, gap: int = 6):
    folder = ROOT / f"fixed_sequence_golden_{scene}_0001" / "depth_truth"
    paths = sorted(folder.glob("frame_*.npz"))
    if not paths:
        raise RuntimeError(f"missing fixed-rig sequence: {folder}")
    for index in range(5, len(paths) - gap, 5):
        yield np.load(paths[index]), np.load(paths[index + gap])


def floor_appearance(scene: str, camera: str, gap: int = 6):
    records = []
    for first, second in paired_frames(scene, gap):
        rotation0, center0 = camera_geometry(first[f"{camera}_camera_usd_world"])
        rotation1, center1 = camera_geometry(second[f"{camera}_camera_usd_world"])
        rows, cols = np.indices((480, 640))
        rows, cols = rows[::4, ::4], cols[::4, ::4]
        rays_camera = np.stack((
            (cols - CX) / FOCAL_LENGTH_PX,
            (rows - CY) / FOCAL_LENGTH_PX,
            np.ones_like(cols),
        ), axis=-1)
        rays_world = rays_camera @ rotation0.T
        with np.errstate(divide="ignore", invalid="ignore"):
            floor_depth = -center0[2] / rays_world[..., 2]
        floor_points = center0 + floor_depth[..., None] * rays_world
        mapped_u, mapped_v, mapped_depth = project_world(floor_points, rotation1, center1)
        in_frame = (
            (floor_depth > 0.45) & (floor_depth < 8.0)
            & (mapped_depth > 0.1)
            & (mapped_u >= 2) & (mapped_u < 638)
            & (mapped_v >= 2) & (mapped_v < 478)
        )
        truth = first[f"{camera}_depth_m"][rows, cols]
        hit_world = center0 + np.where(np.isfinite(truth), truth, 0.0)[..., None] * rays_world
        root_center = np.asarray(first["root_position_world_m"], dtype=np.float64)
        outside_robot = np.linalg.norm(hit_world - root_center, axis=-1) > 0.55
        scene_pixel = np.isfinite(truth) & (truth >= 0.45) & outside_robot
        ground = scene_pixel & (np.abs(truth - floor_depth) < 0.12)
        above_ground = scene_pixel & (truth < floor_depth - 0.25)
        rgb0 = cv2.cvtColor(first[f"{camera}_rgb"], cv2.COLOR_RGB2GRAY).astype(np.float32)
        rgb1 = cv2.cvtColor(second[f"{camera}_rgb"], cv2.COLOR_RGB2GRAY).astype(np.float32)
        remap_x = mapped_u.astype(np.float32)
        remap_y = mapped_v.astype(np.float32)
        warped = cv2.remap(rgb1, remap_x, remap_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        residual = np.abs(rgb0[::4, ::4] - warped)
        # Comparing only pixels with some texture avoids counting black regions
        # as confident floor observations.
        local_variance = cv2.boxFilter(rgb0 * rgb0, -1, (7, 7)) - cv2.boxFilter(rgb0, -1, (7, 7)) ** 2
        textured = local_variance[::4, ::4] > 9.0
        choose = in_frame & textured & (ground | above_ground)
        labels = above_ground[choose]
        scores = residual[choose]
        records.append({
            "time_s": float(first["simulation_time_s"]),
            "camera_translation_m": float(np.linalg.norm(center1 - center0)),
            "ground_labeled_pixels": int((choose & ground).sum()),
            "above_ground_labeled_pixels": int((choose & above_ground).sum()),
            "median_floor_residual_gray": float(np.median(residual[choose & ground])) if np.any(choose & ground) else None,
            "median_above_ground_residual_gray": float(np.median(residual[choose & above_ground])) if np.any(choose & above_ground) else None,
            "auc_above_ground_from_residual": binary_auc(labels, scores),
        })
    return records


def motion_features(scene: str, camera: str, gap: int = 6):
    e0 = np.load(ROOT / "depth_audit_control_golden_e0_0001" / "depth_truth" / "frame_000.npz")
    near_self = np.isfinite(e0[f"{camera}_depth_m"]) & (e0[f"{camera}_depth_m"] < 0.45)
    records = []
    for first, second in paired_frames(scene, gap):
        grayscale0 = cv2.cvtColor(first[f"{camera}_rgb"], cv2.COLOR_RGB2GRAY)
        grayscale1 = cv2.cvtColor(second[f"{camera}_rgb"], cv2.COLOR_RGB2GRAY)
        points0 = cv2.goodFeaturesToTrack(
            grayscale0, maxCorners=800, qualityLevel=0.005, minDistance=5,
            mask=np.uint8(~near_self) * 255,
        )
        if points0 is None:
            records.append({"time_s": float(first["simulation_time_s"]), "detected": 0})
            continue
        points1, status, _ = cv2.calcOpticalFlowPyrLK(
            grayscale0, grayscale1, points0, None, winSize=(21, 21), maxLevel=3,
        )
        back, back_status, _ = cv2.calcOpticalFlowPyrLK(
            grayscale1, grayscale0, points1, None, winSize=(21, 21), maxLevel=3,
        )
        p0 = points0.reshape(-1, 2)
        p1 = points1.reshape(-1, 2)
        valid_track = (
            status.reshape(-1).astype(bool) & back_status.reshape(-1).astype(bool)
            & (np.linalg.norm(back.reshape(-1, 2) - p0, axis=1) < 1.0)
        )
        rr0 = np.clip(np.rint(p0[:, 1]).astype(int), 0, 479)
        cc0 = np.clip(np.rint(p0[:, 0]).astype(int), 0, 639)
        rr1 = np.clip(np.rint(p1[:, 1]).astype(int), 0, 479)
        cc1 = np.clip(np.rint(p1[:, 0]).astype(int), 0, 639)
        valid_track &= ~near_self[rr1, cc1]
        truth = first[f"{camera}_depth_m"][rr0, cc0]
        scene_track = valid_track & np.isfinite(truth) & (truth > 0.45) & (truth < 10.0)
        rotation0, center0 = camera_geometry(first[f"{camera}_camera_usd_world"])
        rotation1, center1 = camera_geometry(second[f"{camera}_camera_usd_world"])
        rays = np.column_stack((
            (p0[:, 0] - CX) / FOCAL_LENGTH_PX,
            (p0[:, 1] - CY) / FOCAL_LENGTH_PX,
            np.ones(len(p0)),
        ))
        safe_truth = np.where(np.isfinite(truth), truth, 0.0)
        world = center0 + (rays * safe_truth[:, None]) @ rotation0.T
        expected_u, expected_v, expected_depth = project_world(world, rotation1, center1)
        projected_error = np.hypot(expected_u - p1[:, 0], expected_v - p1[:, 1])
        physical_inlier = scene_track & (expected_depth > 0.1) & (projected_error < 3.0)
        records.append({
            "time_s": float(first["simulation_time_s"]),
            "camera_translation_m": float(np.linalg.norm(center1 - center0)),
            "detected": int(len(points0)),
            "round_trip_tracks": int(valid_track.sum()),
            "tracks_on_renderer_scene": int(scene_track.sum()),
            "geometrically_correct_tracks": int(physical_inlier.sum()),
            "geometrically_correct_fraction_of_scene_tracks": float(physical_inlier.sum() / max(scene_track.sum(), 1)),
        })
    return records


def central_renderer_visibility(scene: str):
    """Count visible central scene samples, solely as an offline oracle label."""
    folder = ROOT / f"fixed_sequence_golden_{scene}_0001" / "depth_truth"
    records = []
    for index in (0, 10, 20, 30, 40, 50):
        path = folder / f"frame_{index:03d}.npz"
        if not path.exists():
            continue
        frame = np.load(path)
        root = np.asarray(frame["root_position_world_m"], dtype=np.float64)
        w, x, y, z = np.asarray(frame["root_quaternion_world_wxyz"], dtype=np.float64)
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        cosine, sine = np.cos(yaw), np.sin(yaw)
        count = 0
        ranges = []
        for camera in ("left", "right"):
            depth = frame[f"{camera}_depth_m"][::4, ::4]
            rows, cols = np.indices((480, 640))
            rows, cols = rows[::4, ::4], cols[::4, ::4]
            valid = np.isfinite(depth) & (depth > 0.45) & (depth < 10)
            rotation, center = camera_geometry(frame[f"{camera}_camera_usd_world"])
            rays = np.stack((
                (cols - CX) / FOCAL_LENGTH_PX,
                (rows - CY) / FOCAL_LENGTH_PX,
                np.ones_like(cols),
            ), axis=-1)
            hits = center + (np.where(valid, depth, 0)[..., None] * rays) @ rotation.T
            dx, dy = hits[..., 0] - root[0], hits[..., 1] - root[1]
            forward = cosine * dx + sine * dy
            lateral = -sine * dx + cosine * dy
            from_center = np.linalg.norm(hits - root, axis=-1)
            central = (
                valid & (from_center > 0.55)
                & (forward > 0.6) & (forward < 4.0)
                & (np.abs(lateral) < 0.4)
                & (hits[..., 2] > 0.1) & (hits[..., 2] < 1.5)
            )
            count += int(central.sum())
            ranges.extend(forward[central].tolist())
        records.append({
            "time_s": float(frame["simulation_time_s"]),
            "central_scene_samples_every_fourth_pixel": count,
            "minimum_forward_range_m": float(min(ranges)) if ranges else None,
            "median_forward_range_m": float(np.median(ranges)) if ranges else None,
        })
    return records


def main():
    result = {}
    for scene in ("e2", "e1"):
        result[scene] = {"central_renderer_visibility": central_renderer_visibility(scene)}
        for camera in ("left", "right"):
            result[scene][camera] = {
                "motion_features": motion_features(scene, camera),
                "floor_appearance": floor_appearance(scene, camera),
            }
    output = ROOT / "fixed_temporal_audit.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
