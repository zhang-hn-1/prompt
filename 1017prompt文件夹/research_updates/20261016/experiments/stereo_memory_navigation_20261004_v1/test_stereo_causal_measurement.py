"""Hand-derived cases catch epipolar sign, invented support and ambiguity errors."""
import numpy as np
import stereo_causal_measurement as scm


def test_visibility_partitions_mask_occlusion_and_missing_depth_without_free_invention():
    # Native positive disparity shifts the true right partner UP by one pixel.
    z = np.full((6, 5), 2.)
    right = np.full_like(z, 2.)
    lm = np.zeros_like(z, bool); rm = lm.copy()
    lm[4, 0] = True; rm[3, 1] = True
    right[3, 2] = 1.; right[3, 3] = np.nan
    labels = scm.visibility_partition(z, right, lm, rm, focal_baseline=2.)
    assert labels[4].tolist() == [1, 3, 4, 5, 6]
    assert labels[0, 4] == 2
    assert (labels == 6).sum() == 21


def test_patch_search_obeys_positive_horizontal_disparity_and_recovers_unique_texture():
    rng = np.random.default_rng(2)
    left = rng.integers(0, 256, (9, 30, 3)).astype(float)
    right = np.zeros_like(left); right[:, :-4] = left[:, 4:]
    mask = np.zeros(left.shape[:2], bool)
    costs, texture = scm.patch_costs(left, right, mask, mask, [(15, 4)], radius=2, max_disparity=8)
    assert np.argmin(costs[0]) == 4
    assert costs[0, 4] == 0
    assert texture[0] > 20


def test_patch_search_never_accepts_masked_or_outside_partner():
    image = np.zeros((9, 12, 3))
    lm = np.zeros((9, 12), bool); rm = lm.copy(); rm[4, 3] = True
    costs, _ = scm.patch_costs(image, image, lm, rm, [(5, 4)], radius=2, max_disparity=8)
    assert np.isinf(costs[0, 0:5]).all()  # all cross the masked point
    assert np.isinf(costs[0, 4:]).all()  # disparity >=4 is outside


def test_uniform_patch_is_reported_as_ambiguous_not_unique():
    image = np.full((9, 30, 3), 100.)
    mask = np.zeros((9, 30), bool)
    costs, texture = scm.patch_costs(image, image, mask, mask, [(20, 4)], radius=2, max_disparity=10)
    metrics = scm.cost_metrics(costs, [4.], tolerance=1., exclusion_px=3)
    assert texture[0] == 0
    assert metrics['rival_count'][0] == 6  # 0,1,7,8,9,10
    assert metrics['ambiguous'][0]
    assert metrics['true_minus_best_rival'][0] == 0


def test_unmeasurable_true_patch_cannot_turn_into_an_ambiguity_pass():
    costs = np.array([[0., np.inf, 2., 3.]])
    metrics = scm.cost_metrics(costs, [1.], tolerance=1., exclusion_px=3)
    assert not metrics['true_measurable'][0]
    assert not metrics['ambiguous'][0]
    assert np.isnan(metrics['true_cost'][0])


def test_warp_patch_rejects_left_self_mask_even_when_all_right_partners_supported():
    photo = np.zeros((11, 12)); support = np.ones_like(photo, bool)
    left_mask = np.zeros_like(photo, bool); left_mask[4, 2] = True
    disparity = np.full_like(photo, 4.)
    costs, span = scm.warp_patch_measurements(photo, support, left_mask, disparity, [(4, 4), (7, 4)], radius=2)
    assert np.isnan(costs[0]) and np.isnan(span[0])
    assert costs[1] == 0 and span[1] == 0


def test_warp_patch_exposes_curved_disparity_span_separately_from_photo_residual():
    y, x = np.indices((11, 12))
    photo = np.zeros((11, 12)); support = np.ones_like(photo, bool)
    costs, span = scm.warp_patch_measurements(photo, support, np.zeros_like(support), 3.+y+x/2., [(5, 5)], radius=2)
    assert costs[0] == 0
    assert span[0] == 6  # Independent endpoints: (7+7/2) - (3+3/2).


def test_fractional_true_disparity_uses_nearest_integer_cost_and_float_rival_distance():
    # Characterization of the existing discrete diagnostic, not subpixel truth matching.
    metrics = scm.cost_metrics(np.array([[10., 0., 10., 10., 10., 2., 10., 0., 10.]]), [4.6], 1., 3)
    assert metrics['true_cost'][0] == 2
    assert metrics['rival_count'][0] == 1  # d=1 eligible; d=7 only 2.4px away.
    assert metrics['true_minus_best_rival'][0] == 2


def test_empty_patch_group_returns_empty_arrays_without_fake_pass():
    metrics = scm.cost_metrics(np.empty((0, 417)), [], 1., 3)
    assert metrics['true_measurable'].size == 0
    assert metrics['ambiguous'].size == 0
