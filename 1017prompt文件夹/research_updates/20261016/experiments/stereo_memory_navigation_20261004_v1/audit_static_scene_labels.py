"""Independent source rays vs archived labels; no ray/depth fitting or model call."""
from pathlib import Path
import datetime
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scene_ray_geometry import camera_rays, cast_scene
from stereo_geometry import FOCAL

H = Path(__file__).resolve().parent


def metric(error):
    finite = np.isfinite(error)
    return dict(n=int(error.size), finite_n=int(finite.sum()),
        within_0p01m=float((error <= .01).mean()) if error.size else None,
        within_0p10m=float((error <= .1).mean()) if error.size else None,
        finite_p90_m=float(np.percentile(error[finite], 90)) if finite.any() else None,
        finite_peak_m=float(np.max(error[finite])) if finite.any() else None)


def main():
    protocol = json.loads((H/'scene_ray_protocol.json').read_text())
    for file, sha in protocol['input_sha256'].items():
        assert hashlib.sha256(Path(file).read_bytes()).hexdigest() == sha, file
    geometry = json.loads((H/'static_scene_geometry.json').read_text()); shapes = geometry['shapes']
    masks = np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    physics = np.load(H/'capture_transaction_development/hard_0_e1_0004/physics.npz')
    rows = []; archive = {}; pooled = {}; start = datetime.datetime.now(datetime.timezone.utc).isoformat()
    manifest = json.loads((H/'transaction_development_manifest.json').read_text())
    for row in (r for r in manifest['samples'] if r['sequence'] == protocol['sequence']):
        truth = np.load(row['truth_file']); frame = int(Path(row['rgb_file']).stem.split('_')[1])
        for eye in protocol['eyes']:
            pose = truth[eye+'_camera_usd_world']
            origin, rays, yy, xx = camera_rays(pose, 480, 640, FOCAL, 319.5, 239.5, protocol['pixel_stride'])
            expected, ids = cast_scene(origin, rays, shapes, 3., ground_z=geometry['ground_z_m'])
            norm = np.linalg.norm(rays, axis=-1)
            with np.errstate(invalid='ignore'):
                world = origin+expected[..., None]*rays
                external = np.linalg.norm(world-truth['root_position_world_m'], axis=-1) >= .55
            external &= ~masks[eye][yy, xx] & np.isfinite(expected) & (expected >= .5) & (expected <= 3.)
            body = external & (world[..., 2] >= .05) & (world[..., 2] <= 1.5)
            sphere = np.zeros(ids.shape, bool); incidence = np.full(ids.shape, np.nan)
            for i, shape in enumerate(shapes):
                if shape['type'] != 'Sphere': continue
                matched = ids == i; sphere |= matched
                incidence[matched] = abs(np.sum((world[matched]-shape['centre'])*rays[matched], axis=-1))/(shape['radius']*norm[matched])
            core = body & sphere & (incidence >= protocol['sphere_core_incidence_min'])
            floor = external & (ids == -2)
            # The previous measured root state is only a retrospective alternative label.
            # It never changes the current-pose gate or input to inference/control.
            previous_pose = None; current_reconstruction_error = None
            usd_from_optical = np.diag([1., -1., -1., 1.])
            for k in [frame, max(0, frame-1)]:
                index = k*5
                base = np.eye(4); q = physics['root_quaternion_world_wxyz'][index]
                base[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
                base[:3, 3] = physics['root_position_world_m'][index]
                reconstructed = base @ truth[eye+'_optical_to_base'] @ usd_from_optical
                if k == frame: current_reconstruction_error = float(np.max(abs(reconstructed-pose)))
                if k == max(0, frame-1): previous_pose = reconstructed
            prev_origin, prev_rays, _, _ = camera_rays(previous_pose, 480, 640, FOCAL, 319.5, 239.5, protocol['pixel_stride'])
            previous_expected, previous_ids = cast_scene(prev_origin, prev_rays, shapes, 3., geometry['ground_z_m'])
            comparisons = {}
            for version, label, prediction in [
                ('current_pose_transaction', truth[eye+'_depth_m'][yy, xx], expected),
                ('current_pose_pretransaction', truth[eye+'_pretransaction_depth_m'][yy, xx], expected),
                ('previous_pose_transaction', truth[eye+'_depth_m'][yy, xx], previous_expected)]:
                with np.errstate(invalid='ignore'):
                    error = abs(label-prediction)*norm
                error[~np.isfinite(error) | (label <= 0)] = np.inf
                comparisons[version] = dict(sphere_core=metric(error[core]), floor=metric(error[floor]))
                if version == 'current_pose_transaction':
                    archive[f'{eye}_{frame:03d}_current_ray_error_m'] = error
                for group, selected in [('sphere_core', core), ('floor', floor)]:
                    pooled.setdefault((eye, version, group), []).append(error[selected])
            actual_core = comparisons['current_pose_transaction']['sphere_core']
            available = actual_core['n'] >= protocol['geometric_gate']['min_labels_per_supported_eye_frame']
            passed = available and actual_core['within_0p01m'] >= .9 and actual_core['finite_p90_m'] <= .01
            per_shape = []
            for i in np.unique(ids[core]):
                selected = core & (ids == i)
                per_shape.append(dict(path=shapes[int(i)]['path'], n=int(selected.sum()),
                    actual=metric(archive[f'{eye}_{frame:03d}_current_ray_error_m'][selected])))
            rows.append(dict(frame=frame, eye=eye, episode_time_s=float(truth['simulation_time_s']),
                independent_core_n=int(core.sum()), label_gate_applicable=available, label_gate_passed=bool(passed),
                current_root_to_usd_pose_reconstruction_max_abs=current_reconstruction_error,
                previous_pose_is_initial_duplicate=frame == 0, comparisons=comparisons, sphere_objects=per_shape,
                floor_within_authored_xy_bounds=bool(np.all((world[floor, :2] >= geometry['ground_xy_min_m']) &
                    (world[floor, :2] <= geometry['ground_xy_max_m'])))))
            prefix = f'{eye}_{frame:03d}'
            for key, values in [('expected_axis_depth_m', expected), ('expected_scene_id', ids),
                ('sphere_core_selection', core), ('floor_selection', floor), ('sphere_incidence', incidence),
                ('previous_pose_expected_axis_depth_m', previous_expected), ('previous_pose_scene_id', previous_ids)]:
                archive[prefix+'_'+key] = values
            print('STATIC_RAY', frame, eye, actual_core, 'pass', passed, flush=True)
    result = dict(role=protocol['role'], started_utc=start,
        ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), rows=rows,
        groups={eye:{version:{group:metric(np.concatenate(v)) for (e, ver, group),v in pooled.items()
            if e == eye and ver == version} for version in ['current_pose_transaction','current_pose_pretransaction','previous_pose_transaction']}
            for eye in protocol['eyes']},
        applicable_cases=sum(r['label_gate_applicable'] for r in rows),
        geometric_label_gate_passed=bool(any(r['label_gate_applicable'] for r in rows) and
            all(r['label_gate_passed'] for r in rows if r['label_gate_applicable'])),
        rgb_gate_unchanged=True, rgb_navigation_admitted=False, new_simulations=0, new_inference_calls=0,
        limitations='Static authored analytic primary geometry plus checked child dimensions, not exact subdivided renderer equivalence. Fixed body masks may omit real self-occlusion. RGB contour overlay is visual, not automatic semantic validation. Previous pose never replaces current gate or goes online.',
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'scene_ray_geometry.py',H/'static_scene_geometry.json',H/'scene_ray_protocol.json']})
    np.savez_compressed(H/'static_scene_ray_measurements.npz', **archive)
    (H/'static_scene_label_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for file, sha in protocol['input_sha256'].items():
        assert hashlib.sha256(Path(file).read_bytes()).hexdigest() == sha, file
    print('COMPLETE', result['geometric_label_gate_passed'], result['applicable_cases'])


if __name__ == '__main__': main()
