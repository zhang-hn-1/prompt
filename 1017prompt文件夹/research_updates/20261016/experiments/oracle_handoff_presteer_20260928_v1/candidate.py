"""Prepare known next steering while first handoff still obeys B0 stop rule."""
from probe import EventOracle
class PresteerOracle(EventOracle):
    def _zero_action(self,source='zero_request'):
        if source=='direction_change_braking' and self.handoff_index is None:
            waypoint=self._reference[self._path_index]
            self._tracking_audit.update(presteer_active=True,presteer_reference_index=int(self._path_index))
            return self._request_action(0.,float(waypoint.auxiliary_angle_rad),source=source)
        return super()._zero_action(source)
