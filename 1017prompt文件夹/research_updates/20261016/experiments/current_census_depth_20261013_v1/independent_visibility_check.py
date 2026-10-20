"""Review-only check of the 46 preselected post-seal visibility rows."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import sys
import cv2
import numpy as np

D = Path(__file__).resolve().parent
W = D.parents[1]
H = W / 'experiments/stereo_memory_navigation_20261004_v1'
V = W / 'experiments/viewpoint_confirmation_navigation_20261012_v1'
O = W / 'experiments/observed_end_subgoal_20261013_v1'
R = O / 'candidate_e1_run'
sys.path[:0] = [str(V), str(H), str(W / 'experiments/risk_height_navigation_20261012_v1'),
               '/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from rig_adapter import Rig, RigMask

sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text())
assert (D / 'prediction_seal.json').exists()
reference = read(D / 'blocker_visibility_diagnostic.json')
for p, h in reference['input_sha'].items():
    assert sha(p) == h
lookup = {(r['frame'], r['pixel']): r for r in reference['rows']}
cad = RigMask(Rig('forward', .4, .3))
focal = 584.8888888888889
rows = []
maximum_uv_difference = 0.0
maximum_depth_difference = 0.0
maximum_rotation_orthogonality_error = 0.0


def direct_hamming_sum(left, right, left_x, left_y, right_x, right_y):
    answer = 0
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            lx, ly, rx, ry = left_x + dx, left_y + dy, right_x + dx, right_y + dy
            # Center contributes zero; remaining 48 booleans are Census bits.
            a = left[ly-3:ly+4, lx-3:lx+4] < left[ly, lx]
            b = right[ry-3:ry+4, rx-3:rx+4] < right[ry, rx]
            assert a.shape == b.shape == (7, 7)
            answer += int(np.count_nonzero(a != b))
    return answer


def support(mask, x, y):
    return 4 <= x < 636 and 4 <= y < 476 and not bool(mask[y-4:y+5, x-4:x+5].any())


for item in read(O / 'blocker_provenance.json')['source_epochs']:
    index = int(item['frame'].split('_')[1])
    filename = f'frame_{index:03d}.npz'
    with np.load(R / 'frames' / filename) as frame, np.load(R / 'history' / filename) as history, np.load(D / 'predictions/C54' / filename) as pred:
        rotation = frame['right_camera_usd_world'][:3, :3].astype(float)
        maximum_rotation_orthogonality_error = max(maximum_rotation_orthogonality_error, float(np.max(abs(rotation.T @ rotation - np.eye(3)))))
        masks = cad.project(history['proprio_history'][-1, 9:12])
        left = cv2.cvtColor(frame['left_rgb'], cv2.COLOR_RGB2GRAY)
        right = cv2.cvtColor(frame['right_rgb'], cv2.COLOR_RGB2GRAY)
        lr, rr = np.rot90(left), np.rot90(right)
        disparity = np.rot90(pred['disparity_px'], -1)
        best = np.rot90(pred['cost_best'], -1)
        for pixel in item['pixel_ids']:
            y, x = divmod(pixel, 640)
            z = float(frame['left_depth_m'][y, x])
            ray = np.array([(x - 319.5) * z / focal, -(y - 239.5) * z / focal, -z, 1.0])
            world = frame['left_camera_usd_world'] @ ray
            right_usd = np.linalg.solve(frame['right_camera_usd_world'], world)
            right_optical = right_usd[:3] * np.array([1, -1, -1])
            uv = focal * right_optical[:2] / right_optical[2] + [319.5, 239.5]
            rx, ry = np.rint(uv).astype(int)
            expected = lookup[(index, pixel)]
            maximum_uv_difference = max(maximum_uv_difference, float(np.max(abs(uv - expected['true_partner_uv']))))
            # Saved rotations have finite precision. Full inverse and the
            # production transpose differ microscopically, below 0.0001 px.
            np.testing.assert_allclose(uv, expected['true_partner_uv'], rtol=0, atol=1e-4)
            assert rx == x, 'native epipolar x must agree at nearest-pixel precision'
            # Independently confirm the sign and fB/Z relationship on actual poses.
            np.testing.assert_allclose(y - uv[1], focal * .6 / z, rtol=0, atol=1e-4)
            inside = bool(0 <= rx < 640 and 0 <= ry < 480 and right_optical[2] > 0)
            rz = float(frame['right_depth_m'][ry, rx])
            gap = rz - right_optical[2]
            good = bool(np.isfinite(rz) and rz > 0)
            maximum_depth_difference = max(maximum_depth_difference, abs(gap - expected['right_depth_gap_m']))
            assert inside == expected['inside']
            assert good == expected['right_GT_finite']
            assert np.isclose(gap, expected['right_depth_gap_m'], rtol=0, atol=1e-6)
            compatible = bool(inside and good and abs(gap) <= .15)
            assert compatible == expected['right_depth_compatible_with015m']
            clear_left = support(masks['left'], x, y)
            clear_right = support(masks['right'], rx, ry)
            assert clear_left == expected['left_9x9_current_CAD_clear']
            assert clear_right == expected['right_9x9_current_CAD_clear']
            chosen_y = int(np.rint(y - float(disparity[y, x])))
            tc = direct_hamming_sum(left, right, x, y, rx, ry)
            cc = direct_hamming_sum(left, right, x, y, x, chosen_y)
            tc_rotated = direct_hamming_sum(lr, rr, y, 639-x, ry, 639-rx)
            cc_rotated = direct_hamming_sum(lr, rr, y, 639-x, chosen_y, 639-x)
            assert tc == tc_rotated == expected['true_integer_cost']
            assert cc == cc_rotated == expected['chosen_integer_cost'] == int(best[y, x])
            rows.append(dict(frame=index, pixel=pixel, inside=inside, compatible=compatible,
                             left_clear=clear_left, right_clear=clear_right, true_integer_cost=tc,
                             chosen_integer_cost=cc, chosen_is_saved_argmin_cost=True))
summary = dict(total=len(rows), inside=sum(r['inside'] for r in rows),
               GT_right_compatible=sum(r['compatible'] for r in rows),
               left_current_CAD_clear=sum(r['left_clear'] for r in rows),
               right_current_CAD_clear=sum(r['right_clear'] for r in rows),
               true_cost_above_chosen=sum(r['true_integer_cost'] > r['chosen_integer_cost'] for r in rows))
assert summary == reference['summary']
report = dict(utc=datetime.now(timezone.utc).isoformat(), decision='POSTSEAL_46_QUERY_DIAGNOSTIC_ARITHMETIC_CONFIRMED',
              summary=summary, maximum_uv_difference=maximum_uv_difference,
              maximum_right_depth_gap_difference=maximum_depth_difference,
              maximum_saved_rotation_orthogonality_error=maximum_rotation_orthogonality_error,
              all_46_native_rotated_costs_exact=True, all_46_chosen_costs_equal_saved_argmin=True,
              rows=rows, script_sha256=sha(__file__), diagnostic_sha256=sha(D / 'blocker_visibility_diagnostic.json'),
              interpretation='Within this prior selected cohort, true nearest-pixel correspondences mostly have compatible right rendered depth and all have clear current CAD support, while local ordinal cost favors another match. This supports a local correspondence ambiguity diagnosis only.',
              limitations=['46 posthoc previously selected queries only; no full-scene or sequence visibility conclusion.',
                           'The .15m nearest-pixel right-depth agreement is a coarse compatibility diagnostic, not a subpixel occlusion proof.',
                           '15 candidate outputs are unknown despite having a local integer winner. Cost comparison includes those rejected winners and does not call them accepted measurements.',
                           'CAD geometry projection reuses the unchanged RigMask; support neighborhood checks and both native/rotated Hamming sums are independently calculated.',
                           'No new candidate, parameter change, source admission or navigation implication follows.'])
(D / 'independent_visibility_review.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
closure = read(D / 'independent_closure.json')
closure['posthoc_visibility_review'] = dict(file=str(D / 'independent_visibility_review.json'), sha256=sha(D / 'independent_visibility_review.json'), summary=summary)
closure['visual_review'] = dict(inspected=['figures/C54_depth_495.png', 'figures/C54_maps_899.png'],
                              arithmetic='All 40 selected map metric sets matched in raw closure; inspected C54 final-map titles 142/12/135 and 740/477/679 match verified counts.',
                              limitations='Two of six rendered figures were inspected visually; figure source math was inspected. Titles state fixed trajectory and unknown-space semantics.')
(D / 'independent_closure.json').write_text(json.dumps(closure, indent=2, allow_nan=False) + '\n')
print(json.dumps(dict(summary=summary, max_uv_error=maximum_uv_difference, max_gap_error=maximum_depth_difference)))
