"""Regression for closed-node metadata mutation invalidating descendants."""
import numpy as np
import reserve_planner
from source_map import SourceMapReplanner,OLD
from scripts.run_round4_fixed_reference import load_transformed_reference
from types import SimpleNamespace
from test_reserve import MODEL

def test_reconstructed_path_preserves_each_expanded_motion_primitive(monkeypatch):
    monkeypatch.setattr(reserve_planner,'ESTIMATED_TIME_BUDGET_S',float('inf'))
    ref,_=load_transformed_reference(OLD/'episode_hard_0_e1_0004/states.npz',case_id='0004')
    fn=SourceMapReplanner('0004',MODEL)
    result=fn(SimpleNamespace(position_world_xy=ref.points_xy[0],yaw_world_rad=float(ref.yaw_rad[0])),ref.points_xy[-1])
    assert result.status=='success'
    for a,b in zip(result.waypoints,result.waypoints[1:]):
        curvature=MODEL.curvature(a.main_command_rad_s,a.auxiliary_angle_rad)
        sign=1 if a.drive_direction=='forward' else -1
        expected=reserve_planner.sample_motion_primitive(start_pose=(a.x_m,a.y_m,a.yaw_rad),signed_arc_length_m=.3*sign,curvature_1_m=curvature,sample_spacing_m=.05)[-1]
        residual=expected-np.array([b.x_m,b.y_m,b.yaw_rad]);residual[2]=(residual[2]+np.pi)%(2*np.pi)-np.pi
        np.testing.assert_allclose(residual,0.,atol=1e-8)
