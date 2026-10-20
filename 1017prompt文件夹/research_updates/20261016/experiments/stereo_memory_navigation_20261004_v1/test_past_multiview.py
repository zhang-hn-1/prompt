"""Literal geometry and causal fixtures: catch signs, partial support and future leakage."""
import numpy as np
import past_multiview as pv


def view(depth=2., time=0., frame=0):
    return dict(time_s=time,frame=frame,left_pose=np.eye(4),right_pose=np.eye(4),
                left_depth=np.full((5,5),depth),right_depth=np.full((5,5),depth),
                predicted_left=np.full((5,5),depth),confidence_left=np.ones((5,5),bool))


def test_native_usd_forward_and_behind_fov_are_distinguished():
    points=np.array([[0,0,-2],[1,0,-2],[4,0,-2],[0,0,2],[np.nan,0,-2]])
    out=pv.measure_view(points,np.eye(4),np.full((5,5),2.),np.zeros((5,5),bool),2.,2.,2.,.01)
    np.testing.assert_array_equal(out['status'],[5,5,1,0,0])
    np.testing.assert_allclose(out['xy'][:2],[[2,2],[3,2]])


def test_translated_rotated_camera_uses_saved_column_pose():
    pose=np.array([[0,-1,0,5],[1,0,0,-2],[0,0,1,1],[0,0,0,1]],float)
    out=pv.measure_view([[5,-3,-1]],pose,np.full((5,5),2.),np.zeros((5,5),bool),2.,2.,2.,.01)
    np.testing.assert_allclose(out['xy'],[[1,2]])
    np.testing.assert_array_equal(out['status'],[5])


def test_entire_footprint_rejects_body_invalid_depth_and_occlusion():
    point=[[0,0,-2]];depth=np.full((5,5),2.);mask=np.zeros((5,5),bool)
    mask[3,3]=True
    assert pv.measure_view(point,np.eye(4),depth,mask,2,2,2,.01)['status'][0]==2
    mask[:]=False;depth[3,3]=np.nan
    assert pv.measure_view(point,np.eye(4),depth,mask,2,2,2,.01)['status'][0]==3
    depth[3,3]=1.
    assert pv.measure_view(point,np.eye(4),depth,mask,2,2,2,.01)['status'][0]==4
    depth[3,3]=3.
    assert pv.measure_view(point,np.eye(4),depth,mask,2,2,2,.01)['status'][0]==4


def test_metric_prediction_needs_all_confidence_and_correct_depth_not_visibility_alone():
    args=([[0,0,-2]],np.eye(4),np.full((5,5),2.),np.zeros((5,5),bool),2,2,2,.01)
    pred=np.full((5,5),2.);conf=np.ones((5,5),bool)
    good=pv.measure_view(*args,predicted=pred,confidence=conf,rgb_tolerance=.1)
    assert good['rgb_supported'][0] and good['rgb_accurate'][0]
    pred[3,3]=2.2
    wrong=pv.measure_view(*args,predicted=pred,confidence=conf,rgb_tolerance=.1)
    assert wrong['rgb_supported'][0] and not wrong['rgb_accurate'][0] and wrong['status'][0]==5
    pred[:]=2.;conf[3,3]=False
    assert not pv.measure_view(*args,predicted=pred,confidence=conf,rgb_tolerance=.1)['rgb_supported'][0]


def test_future_and_current_pairs_are_not_read_and_history_source_is_actual_past():
    masks={'left':np.zeros((5,5),bool),'right':np.zeros((5,5),bool)}
    pairs=[view(time=0.,frame=0),dict(time_s=1.,left_pose='poison'),dict(time_s=2.,left_pose='poison')]
    out=pv.measure_history([[0,0,-2]],1.,pairs,masks,2,2,2,.01,.1)
    assert out['any_eye'][0] and out['stereo'][0] and out['rgb_reliable'][0]
    np.testing.assert_array_equal(out['first_any_frame'],[0]);np.testing.assert_allclose(out['first_any_time_s'],[0.])
    np.testing.assert_array_equal(out['first_any_eye_bits'],[3])
    for key,val in out.items():np.testing.assert_equal(val,pv.measure_history([[0,0,-2]],1.,pairs[:1],masks,2,2,2,.01,.1)[key])
    empty=pv.measure_history([[0,0,-2]],0.,pairs,masks,2,2,2,.01,.1)
    assert not empty['any_eye'][0] and empty['first_any_frame'][0]==-1


def test_right_only_past_visibility_is_not_reliable_stereo_depth():
    pair=view();pair['left_depth'][:]=1.
    masks={'left':np.zeros((5,5),bool),'right':np.zeros((5,5),bool)}
    out=pv.measure_history([[0,0,-2]],1.,[pair],masks,2,2,2,.01,.1)
    assert out['any_eye'][0] and not out['stereo'][0] and not out['rgb_reliable'][0]
    np.testing.assert_array_equal(out['first_any_eye_bits'],[2])
