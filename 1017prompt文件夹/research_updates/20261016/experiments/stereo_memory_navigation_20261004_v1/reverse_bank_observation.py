"""One distinct reverse/opposite-bank observation family; no dynamics model."""
import numpy as np
from goal_camera_observation import GoalCameraObservation, current_observer_inputs

class ReverseBankObservation(GoalCameraObservation):
 def request(self, *args, **kwargs):
  parent_result=super().request(*args, **kwargs)
  result=parent_result if self.phase=='await' else np.array([-.3 if self.phase=='turn' else 0.,-self.bank])
  self.records[-1].update(request=result.tolist(),motion_family='REVERSE_OPPOSITE_BANK_UNCALIBRATED_OBSERVATION',not_navigation=True)
  return result
