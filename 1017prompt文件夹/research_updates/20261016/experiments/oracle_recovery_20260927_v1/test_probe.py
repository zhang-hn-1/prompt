import math
import numpy as np
import pytest
from probe import align_reference, canonical_xy, EventOracle
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.policies import EpisodeContext
from training.action_protocol import ActionProtocol

def test_alignment_preserves_shape_commands_and_inverts_coordinates():
    ref=tuple(KinodynamicWaypoint(x, 0., 0., 'forward', .5, .2) for x in (0., 1., 2.))
    out, transform=align_reference(ref, 1, np.array([3.,4.]), math.pi/2)
    assert (out[1].x_m,out[1].y_m)==pytest.approx((3.,4.))
    assert out[1].yaw_rad==pytest.approx(math.pi/2)
    assert np.array([(w.x_m,w.y_m) for w in out])==pytest.approx(np.array([[3.,3.],[3.,4.],[3.,5.]]))
    assert canonical_xy(np.array([(w.x_m,w.y_m) for w in out]),transform)==pytest.approx(np.array([[0.,0.],[1.,0.],[2.,0.]]))
    assert [(w.main_command_rad_s,w.auxiliary_angle_rad) for w in out]==[(.5,.2)]*3

def test_event_requires_real_handoff_then_progress_and_no_pending():
    proto=ActionProtocol(protocol_id='ball_action_v2_aux30',minimum=(-2.,-.5235987756),maximum=(2.,.5235987756),controls_per_block=5)
    p=EventOracle(proto)
    ref=(KinodynamicWaypoint(0.,0.,0.,'reverse',-.5,0.),KinodynamicWaypoint(-.6,0.,0.,'forward',.5,0.),KinodynamicWaypoint(1.,0.,0.,'forward',0.,0.))
    p.reset(EpisodeContext(episode_id='unit',privileged=True,oracle_path_xy=np.array([[w.x_m,w.y_m] for w in ref]),action_protocol_id=proto.protocol_id,kinodynamic_reference=ref))
    assert not p.ready
    p._path_index=1
    for _ in range(5):p._handle_direction_change('forward',0.)
    assert p.handoff_index==1
    assert not p.ready
    p._path_index=2
    assert p.ready
    p._pending_drive_direction='reverse'
    assert not p.ready
