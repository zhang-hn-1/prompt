"""Contract tests for the named, opt-in lighting profile in the project runtime."""

import unittest
from pathlib import Path

from benchmark.isaac_runtime import IsaacRuntimeConfig
from benchmark.lighting import FIXED_CAMERA_FILL_V1, LEGACY_SUN_V1, lamp_positions_world_m
from scripts.run_pointgoal_benchmark import _build_parser, _child_command


class LightingProfileTests(unittest.TestCase):
    def test_lamps_track_episode_location_but_keep_tested_spacing(self):
        e1 = lamp_positions_world_m((-3.3, 0, 0.45), (3.3, 0, 0.45))
        self.assertEqual(e1, ((-3.0, 0.0, 2.2), (0.0, 0.0, 2.2), (3.0, 0.0, 2.2)))
        translated = lamp_positions_world_m((17.0, 2.0, 0.45), (23.0, 2.0, 0.45))
        self.assertEqual(translated, ((17.0, 2.0, 2.2), (20.0, 2.0, 2.2), (23.0, 2.0, 2.2)))

    def test_config_preserves_legacy_by_default_and_rejects_unknown_profile(self):
        base = dict(scene_root=Path('.'), contract_path=Path('contract.yaml'), gui=False, record_video=False)
        self.assertEqual(IsaacRuntimeConfig(**base).lighting_profile, LEGACY_SUN_V1)
        self.assertEqual(IsaacRuntimeConfig(**base, lighting_profile=FIXED_CAMERA_FILL_V1).lighting_profile,
                         FIXED_CAMERA_FILL_V1)
        with self.assertRaises(ValueError):
            IsaacRuntimeConfig(**base, lighting_profile='some_other_profile')

    def test_cli_passes_profile_to_episode_worker(self):
        args = _build_parser().parse_args([
            '--manifest', 'manifest.json', '--split', 'golden', '--policy', 'goal_greedy',
            '--scene-root', 'scenes', '--run-root', 'runs', '--headless', '--record-video',
            '--lighting-profile', FIXED_CAMERA_FILL_V1,
        ])
        command = _child_command(args, 'golden_e1_0001')
        offset = command.index('--lighting-profile')
        self.assertEqual(command[offset + 1], FIXED_CAMERA_FILL_V1)


if __name__ == '__main__':
    unittest.main()
