"""Independent truncated sensor/source replay; no physics or renderer labels."""
from pathlib import Path
import datetime, hashlib, json
import numpy as np
from rolling_shadow import OnlineRollingShadow
from rolling_pose_bridge import RollingPoseBridge
from dense_source_points import point_packet
from stereo_geometry import restore_native
from past_point_rgb_memory import PastPointRGBMemory

H = Path(__file__).resolve().parent
D = H / 'RGB_memory_frozen_pose_stress_v1'
O = H / 'online_RGB_memory_fresh_run_v1'
S = np.load(D / 'ideal_sensor_rows.npz')
cut = 195
end_time = float(np.load(O / 'source_predictions' / f'frame_{cut:03d}.npz')['time_s'])
n = int(np.count_nonzero(S['time_s'] <= end_time))
# Materialize only an available prefix. No dynamic pose labels are opened here.
times = S['time_s'][:n].copy()
gyro = S['gyro_body_rad_s'][:n].copy()
encoder = S['main_encoder_rad_s'][:n].copy()
rolling = OnlineRollingShadow()
bridge = RollingPoseBridge(S['initial_position_world_m'], S['initial_rotation_world'])
memory = PastPointRGBMemory()
cursor = 0
sampled = []
counts = dict(poses=0, pose_fields=0, source_packets=0, source_fields=0,
              memory_updates=0, memory_fields=0, future_pose_rejections=0)
def same(a, b):
    assert set(a) == set(b), (set(a), set(b))
    for k in a:
        assert np.array_equal(a[k], b[k], equal_nan=True), k
    return len(a)

for f in sorted((O / 'source_predictions').glob('*.npz')):
    pred = np.load(f)
    frame, t = int(pred['frame']), float(pred['time_s'])
    if frame > cut:
        break
    while cursor < n and times[cursor] <= t:
        st = float(times[cursor])
        rolling.consume(dict(time_s=st, gyro_body_rad_s=gyro[cursor],
                             main_encoder_rad_s=float(encoder[cursor])),
                        S['initial_normal'] if cursor == 0 else None)
        bridge.consume(rolling.records[-1])
        if cursor % 5 == 0:
            sampled.append(bridge.at(st))
        cursor += 1
    pose = bridge.at(t)
    saved_pose = np.load(D / 'estimated_poses' / f'frame_{frame:03d}.npz')
    counts['pose_fields'] += same(pose, saved_pose)
    counts['poses'] += 1
    try:
        bridge.at(t + .02)
    except ValueError:
        counts['future_pose_rejections'] += 1
    else:
        raise AssertionError('future unavailable pose admitted')
    mask = np.load(O / 'masks' / f'frame_{frame:03d}.npz')
    rgb = np.load(O / 'online_inputs' / f'frame_{frame:03d}.npz')
    current_pose = {k: pose[k] for k in ['left_camera_usd_world',
                                       'right_camera_usd_world',
                                       'root_position_world_m', 'time_s']}
    p = {k: pred[k] for k in ['depth_m', 'confidence', 'frame', 'time_s']}
    p['disparity_native_px'] = restore_native(pred['disparity_px'])
    source = point_packet(p, current_pose, mask['left'], mask['right'])
    counts['source_fields'] += same(source, np.load(D / 'source_packets' / f'source_{frame:03d}.npz'))
    counts['source_packets'] += 1
    hazards = int(((source['world'][:, 2] > .05) & (source['world'][:, 2] <= 1.5)).sum())
    low = len(sampled) >= 5 and all(np.linalg.norm(x['velocity_world_m_s'][:2]) < .05 for x in sampled[-5:])
    if not memory.seeded and 18-1e-6 <= t < 24-1e-6 and low and hazards >= 100:
        memory.seed(source, rgb, t, qualified=True)
    proof = memory.query(rgb, current_pose, mask, t)
    saved = np.load(D / 'memory_predictions' / f'frame_{frame:03d}.npz')
    counts['memory_fields'] += same(proof, {k: saved[k] for k in saved.files if k not in ['frame', 'time_s']})
    counts['memory_updates'] += 1

assert cursor == n and memory.source_frame == 180
result = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
              decision='PREFIX_AND_FUTURE_POSE_GUARD_PASS', prefix_last_frame=cut,
              prefix_sensor_rows=n, counts=counts, renderer_labels_read=False,
              current_dynamic_truth_pose_read=False, archive_wide_future_file_poison=False,
              scope='truncated input replay through 19.5s including nonempty supports; bridge future requests rejected',
              code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              sensor_sha256=hashlib.sha256((D/'ideal_sensor_rows.npz').read_bytes()).hexdigest())
out = D / 'causal_prefix_verification_v1.json'
assert not out.exists()
out.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
