"""One archived-pair diagnostic. No inference, fit, runtime policy, or truth input to a predictor."""
from pathlib import Path
import datetime
import hashlib
import json
import numpy as np
import cv2
from stereo_geometry import FOCAL, BASELINE, rectify_pair, restore_native
from stereo_causal_measurement import visibility_partition, patch_costs, cost_metrics, warp_patch_measurements
from audit_stereo import summarize

H = Path(__file__).resolve().parent


def statistics(a):
    a = np.asarray(a); finite = a[np.isfinite(a)]
    return dict(n=int(a.size), finite_n=int(finite.size),
                median=float(np.median(finite)) if finite.size else None,
                p90=float(np.percentile(finite, 90)) if finite.size else None)


def run():
    protocol = json.loads((H/'stereo_causal_protocol.json').read_text())
    for path, expected in protocol['input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    manifest = json.loads((H/'transaction_development_manifest.json').read_text())
    masks = np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    lm, rm = rectify_pair(masks['left'], masks['right'])
    y, x = np.indices((480, 640))
    rays = np.stack(((x-319.5)/FOCAL, (y-239.5)/FOCAL, np.ones(x.shape)), -1)
    stride = (x % 2 == 0) & (y % 2 == 0)
    pooled = {}; patch_pooled = {}; frames = []; archive = {}
    start = datetime.datetime.now(datetime.timezone.utc).isoformat()
    for number, row in enumerate(manifest['samples']):
        sequence = row['sequence']; name = Path(row['rgb_file']).name
        rgb = np.load(row['rgb_file']); truth = np.load(row['truth_file'])
        pred = np.load(H/'predictions_transaction_development/fast'/sequence/name)
        z = truth['left_depth_m']; pose = truth['left_camera_usd_world']
        with np.errstate(invalid='ignore', divide='ignore'):
            world = pose[:3, 3] + (rays*z[..., None]) @ (pose[:3, :3] @ np.diag([1, -1, -1])).T
            native_dl = restore_native(pred['disparity_px'])
            raw_z = np.where(np.isfinite(native_dl) & (native_dl > 0), FOCAL*BASELINE/native_dl, np.nan)
            raw_err = np.abs(raw_z-z)*np.linalg.norm(rays, axis=-1)
            final_err = np.abs(pred['depth_m']-z)*np.linalg.norm(rays, axis=-1)
            true_d = FOCAL*BASELINE/z
        raw_err[~np.isfinite(raw_err)] = np.inf
        final_err[~np.isfinite(final_err)] = np.inf
        labels = visibility_partition(z, truth['right_depth_m'], masks['left'], masks['right'], FOCAL*BASELINE)
        external = np.isfinite(z) & (z >= .5) & (z <= 3) & ~masks['left'] & stride
        external &= np.linalg.norm(world-truth['root_position_world_m'], axis=-1) >= .55
        groups = {'body': external & (world[..., 2] >= .05) & (world[..., 2] <= 1.5),
                  'floor': external & (np.abs(world[..., 2]) <= .05)}
        # Native true-partner bilinear RGB residual is an OFFLINE LABEL. Never infer from it.
        partner_y = np.nan_to_num(y-true_d, nan=-1000., posinf=-1000., neginf=-1000.).astype(np.float32)
        warped = cv2.remap(rgb['right_rgb'].astype(np.float32), x.astype(np.float32), partner_y,
                           cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        photo = np.abs(rgb['left_rgb'].astype(float)-warped).mean(axis=-1)
        # A stricter bilinear footprint flag only describes this diagnostic, not old eligibility.
        y0 = np.floor(partner_y).astype(int); y1 = y0+1
        safe0 = np.clip(y0, 0, 479); safe1 = np.clip(y1, 0, 479)
        photo_valid = (y0 >= 0) & (y1 < 480) & ~masks['right'][safe0, x] & ~masks['right'][safe1, x]
        left, right = rectify_pair(rgb['left_rgb'], rgb['right_rgb'])
        rect_d = np.rot90(true_d); rect_photo = np.rot90(photo); rect_photo_valid = np.rot90(photo_valid)
        frame = dict(sequence=sequence, frame=name, groups={})
        for group, selection in groups.items():
            common = selection & (labels == 6)
            sample = dict(labels=labels[selection], raw=raw_err[common], final=final_err[common])
            store = pooled.setdefault((sequence, group), {k: [] for k in sample})
            for k, v in sample.items(): store[k].append(v)
            counts = {str(i): int((sample['labels'] == i).sum()) for i in range(7)}
            frame['groups'][group] = dict(left_external_n=int(selection.sum()), visibility_counts=counts,
                raw_shared=summarize(sample['raw']), final_shared=summarize(sample['final']))
            # Deterministic row-major stratification, independent of model outputs.
            eligible = np.argwhere(np.rot90(common))
            if len(eligible) > protocol['max_patch_samples_per_frame_per_group']:
                eligible = eligible[np.linspace(0, len(eligible)-1, protocol['max_patch_samples_per_frame_per_group'], dtype=int)]
            coords = eligible[:, ::-1]
            if not len(coords): continue
            costs, texture = patch_costs(left, right, lm, rm, coords,
                protocol['patch_radius_px'], protocol['max_disparity_px'])
            ry, rx = eligible.T
            d = rect_d[ry, rx]
            metrics = cost_metrics(costs, d, protocol['ambiguity_rival_cost_slack_0_255'],
                protocol['ambiguity_rival_min_distance_px'])
            gt_warp_cost, d_span = warp_patch_measurements(rect_photo, rect_photo_valid, lm, rect_d,
                coords, protocol['patch_radius_px'])
            values = dict(texture_std=texture, true_disparity=d,
                raw_error=np.rot90(raw_err)[ry, rx], final_error=np.rot90(final_err)[ry, rx],
                gt_warp_rgb_cost=gt_warp_cost, true_patch_disparity_span=d_span, **metrics)
            dest = patch_pooled.setdefault((sequence, group), {k: [] for k in values})
            for k, v in values.items(): dest[k].append(v)
            prefix = f'{sequence}__{group}__{Path(name).stem}'
            archive[prefix+'__coordinates_xy'] = coords
            archive[prefix+'__costs'] = costs.astype(np.float32)
            for k, v in values.items(): archive[prefix+'__'+k] = v
            frame['groups'][group]['patch_n'] = len(coords)
        frames.append(frame)
        print('ARCHIVED_PAIR', number+1, sequence, name, flush=True)
    groups_out = {}; patches_out = {}
    previous = json.loads((H/'fast_audit_transaction_development.json').read_text())
    for (sequence, group), data in pooled.items():
        data = {k: np.concatenate(v) for k, v in data.items()}
        n = len(data['labels']); common = int((data['labels'] == 6).sum())
        final = summarize(data['final']); raw = summarize(data['raw'])
        old = previous['groups'][sequence][group+'_overlap']
        assert final == old, (sequence, group, final, old)
        groups_out.setdefault(sequence, {})[group] = dict(left_external_n=n, shared_n=common,
            shared_fraction=common/n if n else None,
            visibility_counts={str(i): int((data['labels'] == i).sum()) for i in range(7)},
            raw_shared=raw, final_shared=final,
            raw_accurate_then_filtered_n=int(((data['raw'] <= .1) & ~np.isfinite(data['final'])).sum()),
            raw_inaccurate_retained_n=int(((data['raw'] > .1) & np.isfinite(data['final'])).sum()))
    for (sequence, group), data in patch_pooled.items():
        a = {k: np.concatenate(v) for k, v in data.items()}
        measured = a['true_measurable']; weak = a['texture_std'] < protocol['weak_texture_std_0_255']
        common_support = measured & np.isfinite(a['gt_warp_rgb_cost'])
        stats = dict(n=len(measured), true_patch_measurable_n=int(measured.sum()),
            texture_std=statistics(a['texture_std']), true_constant_shift_rgb_cost=statistics(a['true_cost']),
            true_warp_rgb_cost=statistics(a['gt_warp_rgb_cost']),
            true_patch_disparity_span=statistics(a['true_patch_disparity_span']),
            true_minus_best_rival=statistics(a['true_minus_best_rival']),
            constant_vs_warp_common_support=dict(n=int(common_support.sum()),
                constant_integer_shift_rgb_cost=statistics(a['true_cost'][common_support]),
                per_pixel_warp_rgb_cost=statistics(a['gt_warp_rgb_cost'][common_support])),
            rival_count=statistics(a['rival_count'][measured]),
            ambiguity_fraction=float(a['ambiguous'][measured].mean()) if measured.any() else None,
            weak_texture_fraction=float(weak[measured].mean()) if measured.any() else None,
            raw_accuracy=float((a['raw_error'] <= .1).mean()),
            final_accuracy=float((a['final_error'] <= .1).mean()), strata={})
        for name, mask in [('weak', measured & weak), ('textured', measured & ~weak),
                           ('ambiguous', measured & a['ambiguous']), ('unambiguous', measured & ~a['ambiguous'])]:
            stats['strata'][name] = dict(n=int(mask.sum()), raw=summarize(a['raw_error'][mask]),
                final=summarize(a['final_error'][mask]),
                true_warp_rgb_cost=statistics(a['gt_warp_rgb_cost'][mask]))
        patches_out.setdefault(sequence, {})[group] = stats
    result = dict(role=protocol['role'], started_utc=start,
        ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        old_gate_reproduced_exact=True, online_admitted=False, inference_calls=0, simulation_runs=0,
        groups=groups_out, patches=patches_out, frames=frames,
        limitations=protocol['limitations'],
        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                       [Path(__file__), H/'stereo_causal_measurement.py', H/'stereo_causal_protocol.json']})
    np.savez_compressed(H/'stereo_causal_patch_measurements.npz', **archive)
    (H/'stereo_causal_measurement.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for path, expected in protocol['input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    print('COMPLETE old gate exact; no changed inputs; patch groups', len(patches_out), flush=True)


if __name__ == '__main__': run()
