"""Bounded observation using the current measured camera optical axis."""
import math
import numpy as np
from neutral_bank_visibility_probe import NeutralBankVisibilityProbe

class CameraHeadingObservationProbe(NeutralBankVisibilityProbe):
    def __init__(self):
        super().__init__()
        self.event_heading = None
        self.camera_heading_goal_reached = False

    def choose(self, now, source_frame, source_time, qualified_hazards,
               handed_off, nominal_request, axis_time, optical_forward_world):
        t, st, at = float(now), float(source_time), float(axis_time)
        if (not np.isfinite(t) or not np.isfinite(st) or t < self.time
                or st > t + 1e-6 or t-st > .5+1e-6 or st < self.last_source_time):
            raise ValueError('causal source/current clock required before payload')
        if not np.isfinite(at) or abs(at-t) > 1e-9:
            raise ValueError('current axis clock required before payload')
        axis = np.asarray(optical_forward_world, float)
        if (axis.shape != (3,) or not np.isfinite(axis).all()
                or abs(float(np.linalg.norm(axis))-1.) > 1e-6):
            raise ValueError('finite unit world camera axis required')
        heading = math.atan2(axis[1], axis[0]) if np.linalg.norm(axis[:2]) > 1e-8 else None
        old_event = self.event_time
        super().choose(t, source_frame, st, qualified_hazards, handed_off, nominal_request)
        if old_event is None and self.event_time is not None:
            self.event_heading = heading
        active = self.event_time is not None and t < self.event_time+6.
        angle = None
        if self.event_heading is not None and heading is not None:
            angle = math.atan2(math.sin(heading-self.event_heading), math.cos(heading-self.event_heading))
            if active and angle >= math.radians(10):
                self.camera_heading_goal_reached = True
        turning = (active and self.event_heading is not None
                   and not self.camera_heading_goal_reached and t < self.event_time+4.)
        action = np.asarray(nominal_request).copy()
        if active:
            action[:] = 0
            if turning:
                action[::2] = .5
                action[1::2] = -math.pi/6
        self.diagnostics.update(
            active=active, finished=self.event_time is not None and not active,
            turning=turning, camera_heading_goal_reached=self.camera_heading_goal_reached,
            event_camera_heading_rad=self.event_heading, current_camera_heading_rad=heading,
            measured_camera_heading_change_rad=angle,
            turn_not_exercised_invalid_event_axis=self.event_time is not None and self.event_heading is None,
            maximum_turn_s=4., maximum_observation_s=6., desired_bank_rad=-math.pi/8,
            physical_response_prediction_used=False, no_source_gate_relaxation=True,
            known_current_camera_pose_privileged=True)
        return action
