import unittest
from acceptance import stop_hold


class TestStopHold(unittest.TestCase):
    def test_entry_sample_does_not_count(self):
        self.assertIsNone(stop_hold([.1] * 5, [.01] * 5, [0,.1,.2,.3,.4], entry_index=0))

    def test_five_post_entry_endpoints(self):
        self.assertEqual(stop_hold([.1]*6, [.01]*6, [0,.1,.2,.3,.4,.5], entry_index=0), .5)

    def test_rebound_resets_run(self):
        self.assertIsNone(stop_hold([.1]*8, [.01,.01,.01,.06,.01,.01,.01,.01],
                                    [i/10 for i in range(8)], entry_index=0))

    def test_leaving_radius_resets_run(self):
        self.assertIsNone(stop_hold([.1,.1,.1,.21,.1,.1,.1,.1], [.01]*8,
                                    [i/10 for i in range(8)], entry_index=0))

    def test_threshold_is_strict(self):
        self.assertIsNone(stop_hold([.2]*6, [.05]*6, [i/10 for i in range(6)], entry_index=0))

    def test_time_gap_is_not_consecutive_sampling(self):
        self.assertIsNone(stop_hold([.1]*6, [.01]*6, [0,.1,.2,.5,.6,.7], entry_index=0))

    def test_no_goal_entry(self):
        self.assertIsNone(stop_hold([.1]*6, [.01]*6, [i/10 for i in range(6)], entry_index=None))


if __name__ == '__main__':
    unittest.main()
