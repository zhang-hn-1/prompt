"""Actual tracker composition regression; archived development geometry, no physics."""
import json
from pathlib import Path
import numpy as np
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.policies import EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from outgoing_lookahead import make_endpoint_oracle
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')

def test_actual_tracker_reverse_cusp_excludes_overlapping_forward_return():
    a=json.loads((H/'E1_route_side_continuation_geometry_v1/assessment.json').read_text())
    query=next(q for q in a['queries'] if q['frame']==155)
    side=next(q for q in query['queries'] if q['name']!='factual')
    points=tuple(KinodynamicWaypoint(**p) for p in side['waypoints'])
    assert points[0].drive_direction=='reverse' and points[1].drive_direction=='forward'
    xy=np.asarray([(p.x_m,p.y_m) for p in points])
    protocol=load_action_protocol(P/'configs/data_contract.yaml')
    tracker=make_endpoint_oracle(protocol)
    assert [c.__name__ for c in type(tracker).__mro__][:5]==['OutgoingEndpoint','ConfidentSpeed','MappedBankOracle','CuspBoundary','OraclePathPolicy']
    tracker.reset(EpisodeContext('development_cusp_regression',True,xy,protocol.protocol_id,points))
    tracker._active_drive_direction='forward'
    tracker._project_to_reference(xy[2]);assert tracker._path_index==0
    tracker._active_drive_direction='reverse'
    d=tracker._project_to_reference(xy[2]);assert tracker._path_index==0 and d>0
    np.testing.assert_array_equal(tracker._lookahead_target(.5),xy[1])
    assert tracker._tracking_audit['lookahead_index']==1
