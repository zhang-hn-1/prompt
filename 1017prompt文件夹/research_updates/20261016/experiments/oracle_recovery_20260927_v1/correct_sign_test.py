from pathlib import Path
import hashlib,json
root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
here=Path(__file__).resolve().parent
path=root/'research/ball_wm/tests/test_round8_tracking_audit.py'
original=path.read_text()
start=original.index('@pytest.mark.xfail(\n    strict=True,\n    reason="frozen Oracle omits')
end=original.index('\ndef test_circle_curvature_is_stable',start)
new='''@pytest.mark.parametrize("direction_sign", (-1.0, 1.0))
@pytest.mark.parametrize("body_curvature", (-0.5, 0.5))
def test_circle_distinguishes_signed_body_and_waypoint_order_curvature(
    direction_sign: float, body_curvature: float
):
    # DriveModel consumes d(yaw)/d(signed body distance), not d(yaw)/d(path length).
    # For a reverse arc, those curvatures have opposite signs. The policy's
    # desired yaw rate must agree with the signed arc used by the planner.
    poses = sample_motion_primitive(
        start_pose=(0.0, 0.0, 0.0),
        signed_arc_length_m=direction_sign * 1.2,
        curvature_1_m=body_curvature,
        sample_spacing_m=0.6,
    )
    direction = "forward" if direction_sign > 0 else "reverse"
    reference = tuple(
        KinodynamicWaypoint(
            float(x), float(y), float(yaw), direction,
            direction_sign * 0.5 if i < 2 else 0.0, 0.0
        )
        for i, (x, y, yaw) in enumerate(poses)
    )
    policy = OraclePathPolicy(_protocol(), lookahead_m=0.5, yaw_rate_feedback_gain_s=0.0)
    policy.reset(_context(reference))
    policy.act(
        _observation(velocity=(direction_sign * 0.2, 0.0)),
        pointgoal_base_xy=(direction_sign * 2.0, 0.0),
    )
    record = policy.tracking_records[-1]
    assert record["requested_curvature_1_m"] == pytest.approx(body_curvature, abs=2e-3)
    spatial_curvature = float(poses[1, 2] - poses[0, 2]) / 0.6
    assert spatial_curvature == pytest.approx(direction_sign * body_curvature)
    assert record["desired_world_yaw_rate_rad_s"] == pytest.approx(
        direction_sign * 0.2 * body_curvature, abs=1e-7
    )

'''
(here/'test_round8_tracking_audit.before.py').write_text(original)
path.write_text(original[:start]+new+original[end:])
(here/'test_correction.json').write_text(json.dumps({'path':str(path),'before_sha256':hashlib.sha256(original.encode()).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'policy_changed':False},indent=2))
