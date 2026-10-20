from gated_speed import with_confident_speed


def test_only_supported_medium_command_with_small_error_accelerates():
    class Frozen:
        def _schedule_main_magnitude(self, observation, *, baseline_magnitude, **kwargs):
            return baseline_magnitude

    policy = with_confident_speed(Frozen)()
    def request(baseline, curvature, error, planned=1.5):
        return policy._schedule_main_magnitude(None, baseline_magnitude=baseline,
            planned_main_command_rad_s=planned, desired_curvature_1_m=curvature,
            cross_track_error_m=error)
    assert request(1.0, .55, .10) == 1.5
    assert request(1.0, .551, .10) == 1.0
    assert request(1.0, .55, .101) == 1.0
    assert request(.5, .3, .01) == .5
    assert request(1.0, .3, .01, planned=1.0) == 1.0
