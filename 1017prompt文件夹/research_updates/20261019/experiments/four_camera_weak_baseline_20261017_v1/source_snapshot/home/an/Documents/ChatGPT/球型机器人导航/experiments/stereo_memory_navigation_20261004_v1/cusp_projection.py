"""Hold only an unaccepted current direction cusp; delegate all other tracking."""
from pathlib import Path
import json
import sys
import numpy as np
from root_direction_consumer import RootDirectionPolicy
from benchmark.policies import OraclePathPolicy


def with_cusp_boundary(base_class):
    class CuspBoundary(base_class):
        def _project_to_reference(self, position_world_xy):
            reference, reference_xy = self._require_reference()
            if self._active_drive_direction not in ('forward', 'reverse'):
                raise ValueError('valid accepted direction required')
            if reference[self._path_index].drive_direction != self._active_drive_direction:
                # The original scan starts at NEXT waypoint. The CURRENT opposite
                # waypoint must survive until the unchanged direction handler accepts it.
                return float(np.linalg.norm(reference_xy[self._path_index] - position_world_xy))
            return super()._project_to_reference(position_world_xy)
    return CuspBoundary


def make_cusp_oracle(protocol):
    # Exact frozen composition; only the Oracle projection base gains the override.
    old = Path(__file__).resolve().parent.parent / 'oracle_feasible_route_20260929_v9/hybrid'
    sys.path.insert(0, str(old))
    from bank_mapping import BankMap, make_policy
    from gated_speed import with_confident_speed
    cls = with_confident_speed(make_policy(with_cusp_boundary(OraclePathPolicy),
                                         BankMap(json.loads((old / 'bank_mapping.json').read_text()))))
    return cls(protocol, lookahead_m=.5)


class CuspProjectionPolicy(RootDirectionPolicy):
    policy_name = 'privileged_depth_cusp_projection_diagnostic'

    def reset(self, context):
        super().reset(context)
        # Construction/episode reset only, never an action-boundary replacement.
        self.tracker = make_cusp_oracle(self.protocol)
        self.tracker.mapping_records = []
