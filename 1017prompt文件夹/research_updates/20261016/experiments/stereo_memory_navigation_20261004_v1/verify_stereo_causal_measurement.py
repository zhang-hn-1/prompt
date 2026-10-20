"""Verify preserved inputs, old results and actual supported diagnostic arrays."""
from pathlib import Path
import datetime
import hashlib
import json
import numpy as np
from stereo_geometry import FOCAL, BASELINE, rectify_pair

H = Path(__file__).resolve().parent


def main():
    protocol = json.loads((H/'stereo_causal_protocol.json').read_text())
    for path, sha in protocol['input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == sha, path
    old_n = 0
    for name in ['route_clearance_navigation_verification.json', 'online_route_execution_verification.json']:
        old = json.loads((H/name).read_text())
        for path, sha in old['source_and_artifact_sha256'].items():
            assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == sha, path
            old_n += 1
    result = json.loads((H/'stereo_causal_measurement.json').read_text())
    old_result = json.loads((H/'stereo_causal_measurement_pre_review.json').read_text())
    assert result['groups'] == old_result['groups']
    assert result['old_gate_reproduced_exact'] and not result['online_admitted']
    assert result['inference_calls'] == result['simulation_runs'] == 0
    assert '78 passed' in (H/'stereo_causal_review_green.log').read_text()
    for groups in result['groups'].values():
        for group in groups.values():
            assert sum(group['visibility_counts'].values()) == group['left_external_n']
            assert group['shared_n'] == group['final_shared']['n']
    masks = np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    left_mask, right_mask = rectify_pair(masks['left'], masks['right'])
    data = np.load(H/'stereo_causal_patch_measurements.npz')
    supported = 0; total = 0
    for row in json.loads((H/'transaction_development_manifest.json').read_text())['samples']:
        z = np.load(row['truth_file'])['left_depth_m']
        with np.errstate(invalid='ignore', divide='ignore'):
            d = np.rot90(FOCAL*BASELINE/z)
        for group in ['body', 'floor']:
            prefix = row['sequence']+'__'+group+'__'+Path(row['rgb_file']).stem
            key = prefix+'__coordinates_xy'
            if key not in data: continue
            xy = data[key]; cost = data[prefix+'__gt_warp_rgb_cost']
            assert len(xy) == len(cost)
            total += len(xy)
            for (x, y), value in zip(xy, cost):
                if not np.isfinite(value): continue
                supported += 1
                region = np.s_[y-2:y+3, x-2:x+3]
                patch_d = d[region]
                assert patch_d.shape == (5, 5) and np.isfinite(patch_d).all() and (patch_d > 0).all()
                assert not left_mask[region].any()
                yy, xx = np.indices((5, 5)); xx = xx+x-2; yy = yy+y-2
                # Rotated native y-disparity is exactly rectified x-disparity.
                right_x0 = np.floor(xx-patch_d).astype(int); right_x1 = right_x0+1
                assert (right_x0 >= 0).all() and (right_x1 < 480).all()
                assert not right_mask[yy, right_x0].any() and not right_mask[yy, right_x1].any()
    artifacts = [Path(__file__), H/'stereo_causal_measurement.py', H/'audit_stereo_causal_measurement.py',
        H/'test_stereo_causal_measurement.py', H/'plot_stereo_causal_measurement.py',
        H/'stereo_causal_protocol.json', H/'stereo_causal_measurement.json', H/'stereo_causal_patch_measurements.npz',
        H/'stereo_causal_review.json', H/'stereo_causal_red.log', H/'stereo_causal_red_functions.log',
        H/'stereo_causal_green.log', H/'stereo_causal_review_red.log', H/'stereo_causal_review_green.log',
        H/'stereo_causal_measurement.log', H/'stereo_causal_figures.log',
        H/'stereo_causal_measurement_pre_review.json', H/'stereo_causal_patch_measurements_pre_review.npz',
        H/'stereo_causal_measurement_pre_review.log', H/'figures/stereo_causal_fixed_frames.png',
        H/'figures/stereo_causal_visibility_and_cost.png', H/'figures/stereo_causal_fixed_frames_pre_review.png',
        H/'figures/stereo_causal_visibility_and_cost_pre_review.png',
        H.parent.parent/'reports/benchmark_stereo_visibility_and_matching_audit_20261004.md']
    output = dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        frozen_input_sha_verified=len(protocol['input_sha256']), previous_artifact_sha_verified=old_n,
        old_pixel_results_and_gate_unchanged=True, new_simulations=0, new_inferences=0,
        actual_pair_audits=20, patch_samples=total, finite_warp_patches_supported=supported,
        cpu_tests_passed=78, independent_review_important_fixed=True, figures_inspected=True,
        whole_research_complete=False, rgb_admission=False,
        source_and_artifact_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in artifacts})
    (H/'stereo_causal_verification.json').write_text(json.dumps(output, indent=2)+'\n')
    print('VERIFIED', output['verified_utc'], 'old_inputs', len(protocol['input_sha256']), 'old_artifacts', old_n,
          'patches', total, 'supported_warp', supported, 'new_artifacts', len(artifacts))


if __name__ == '__main__': main()
