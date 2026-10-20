from speed_schedule import with_moderate_tracking_speed


def test_schedule_increases_only_tracking_cap_without_exceeding_planned_command():
    class Frozen:
        def _schedule_main_magnitude(self, observation, *, baseline_magnitude, **kwargs):
            return baseline_magnitude

    policy = with_moderate_tracking_speed(Frozen)()
    for baseline, planned, expected in ((0.5, 1.5, 1.0), (1.0, 1.5, 1.5), (1.5, 1.5, 1.5), (0.5, 0.5, 0.5)):
        actual = policy._schedule_main_magnitude(None, baseline_magnitude=baseline, planned_main_command_rad_s=planned, desired_curvature_1_m=0.6, cross_track_error_m=0.1)
        assert actual == expected
