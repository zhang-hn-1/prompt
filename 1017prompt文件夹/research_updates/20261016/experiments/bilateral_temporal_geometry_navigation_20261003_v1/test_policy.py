import numpy as np
from benchmark.policies import GoalGreedyPolicy,EpisodeContext,PolicyObservation
from training.action_protocol import ActionProtocol
from policy import BilateralTemporalPolicy
from geometry import camera_pose
from test_geometry import plane_image


def setup():
    protocol=ActionProtocol('ball_action_v2_aux30',(-2.,-np.pi/6),(2.,np.pi/6),5)
    context=EpisodeContext('irrelevant_metadata',False,None,protocol.protocol_id)
    return protocol,context


def obs(xy,left,right):
    return PolicyObservation(stereo_left_rgb=left,stereo_right_rgb=right,
        proprio_history=np.zeros((1,17),dtype=np.float32),action_history=np.zeros((1,10),dtype=np.float32),
        position_world_xy=np.asarray(xy,dtype=float),yaw_world_rad=0.,
        quaternion_world_wxyz_history=np.array([[1,0,0,0]],dtype=np.float32))


def test_unknown_geometry_remains_unverified_and_matches_nominal_action():
    protocol,context=setup();policy=BilateralTemporalPolicy(protocol);b0=GoalGreedyPolicy(protocol)
    policy.reset(context);b0.reset(context)
    rgb=np.zeros((480,640,3),dtype=np.uint8);o=obs([0,0],rgb,rgb)
    assert np.array_equal(policy.act(o,np.array([8.,0.])),b0.act(o,np.array([8.,0.])))
    assert policy.records[-1]['reference_source']=='unverified_nominal_PointGoal'
    assert policy.records[-1]['unknown_space_is_not_free']


def test_right_eye_rgb_alone_reaches_control_and_changes_request():
    protocol,context=setup();active=BilateralTemporalPolicy(protocol);blank=BilateralTemporalPolicy(protocol)
    active.reset(context);blank.reset(context);rgb=np.zeros((480,640,3),dtype=np.uint8)
    changed=False
    for i in range(10):
        xy=np.array([-.8+i*.12,0.]);right=plane_image(*camera_pose(xy,np.array([1.,0,0,0]),'right'))
        a=active.act(obs(xy,rgb,right),np.array([8.,0.]))
        b=blank.act(obs(xy,rgb,rgb),np.array([8.,0.]))
        changed|=not np.array_equal(a,b)
    assert changed and active._path_xy is not None
    assert active.records[-1]['eyes']['right']['depth_predictions']>0
    assert all(r['eyes']['left']['depth_predictions']==0 for r in active.records)


def test_episode_name_is_not_a_decision_input():
    protocol,c=setup();a=BilateralTemporalPolicy(protocol);b=BilateralTemporalPolicy(protocol)
    a.reset(c);b.reset(EpisodeContext('different_scene_name',False,None,protocol.protocol_id))
    rgb=np.zeros((480,640,3),dtype=np.uint8);o=obs([0,0],rgb,rgb)
    assert np.array_equal(a.act(o,[8,0]),b.act(o,[8,0]))
