"""Read both frozen RGB cameras while leaving the validated actions unchanged."""

from __future__ import annotations

import numpy as np

from policy import pilot
from policy_continuous import ContinuousVisionDetourPolicy


class BilateralShadowPolicy(ContinuousVisionDetourPolicy):
    policy_name = "vision_detour_bilateral_shadow_v1"

    def _eye_summary(self, rgb: np.ndarray, eye_id: int) -> dict[str, int]:
        image = np.asarray(rgb)[::4, ::4][None]
        with self._torch.no_grad():
            x = pilot.rgb_input(image, np.array([eye_id]), self._torch.device("cpu"))
            pred = self._model(x).argmax(1).numpy()[0]
        roi = pred[40:100, 40:140]
        return {
            "obstacle_pixels": int((roi == pilot.OBSTACLE).sum()),
            "free_pixels": int((roi == pilot.FREE).sum()),
            "unknown_pixels": int((roi == pilot.UNKNOWN).sum()),
            "image_left_obstacle_pixels": int((roi[:, :50] == pilot.OBSTACLE).sum()),
            "image_right_obstacle_pixels": int((roi[:, 50:] == pilot.OBSTACLE).sum()),
        }

    def act(self, observation, pointgoal_base_xy):
        action = super().act(observation, pointgoal_base_xy)
        left = self._eye_summary(observation.stereo_left_rgb, 0)
        right = self._eye_summary(observation.stereo_right_rgb, 1)
        record = self.records[-1]
        record["left_eye_shadow"] = left
        record["right_eye_shadow"] = right
        record["right_obstacle_pixels"] = right["obstacle_pixels"]
        record["left_count_consistent"] = left["obstacle_pixels"] == record["obstacle_pixels"]
        return action
