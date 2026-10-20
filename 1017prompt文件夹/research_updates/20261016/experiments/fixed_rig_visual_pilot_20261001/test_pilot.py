import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pilot import FREE, OBSTACLE, UNKNOWN, classify_depth_samples, split_scenes


class LabelGeometryTests(unittest.TestCase):
    def test_ground_wall_and_unmeasurable_pixels_remain_distinct(self):
        # USD axes: +X right, -Y down, -Z forward; world forward is +X.
        pose = np.eye(4)
        pose[:3, :3] = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
        pose[:3, 3] = [0, 0, 0.5]
        ground_u = 319.5 - 0.5 * (16.45 / 13.5 * 480)
        uv = np.array([
            [ground_u, 239.5], [319.5, 239.5],
            [319.5, 239.5], [319.5, 239.5],
        ])
        depth = np.array([1.0, 1.0, 0.1, np.inf])
        labels, central = classify_depth_samples(depth, uv, pose, [0, 0, 0.4], 0.0)
        self.assertEqual(labels.tolist(), [FREE, OBSTACLE, UNKNOWN, UNKNOWN])
        self.assertEqual(central.tolist(), [True, True, False, False])

    def test_root_proximity_is_unknown_even_with_wall_height(self):
        pose = np.eye(4)
        pose[:3, :3] = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
        pose[:3, 3] = [0, 0, 0.5]
        labels, _ = classify_depth_samples(
            np.array([0.5]), np.array([[319.5, 239.5]]), pose,
            [0, 0, 0.4], 0.0,
        )
        self.assertEqual(labels.tolist(), [UNKNOWN])


class LeakageTests(unittest.TestCase):
    def test_scene_holdout_is_disjoint(self):
        for holdout in ("e1", "e2"):
            training, testing = split_scenes(holdout)
            self.assertEqual(set(training).intersection(testing), set())
            self.assertEqual(testing, [holdout])
            self.assertEqual(set(training) | set(testing), {"e0", "e1", "e2"})


if __name__ == "__main__":
    unittest.main()
