import importlib.util
from pathlib import Path
import numpy as np
import pytest

HERE=Path(__file__).parent

def adapter():
    p=HERE/'rig_inputs.py'
    assert p.exists(),'new-rig input validator missing'
    spec=importlib.util.spec_from_file_location('rig_inputs_under_test',p)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

def capture():
    return dict(left_optical_to_base=np.array([[0,0,1,.2],[0,-1,0,.3],[1,0,0,.05],[0,0,0,1.]]),
                right_optical_to_base=np.array([[0,0,1,.2],[0,-1,0,-.3],[1,0,0,.05],[0,0,0,1.]]),
                left_rgb=np.zeros((480,640,3),np.uint8),right_rgb=np.zeros((480,640,3),np.uint8),
                raw_time_s=1.0,left_render_time_s=1.0,right_render_time_s=1.0)


def test_accept_only_measured_current_rig():
    assert adapter().validate_capture(capture()) is True


def test_old_rig_rejected():
    row=capture()
    for eye in ('left','right'):row[eye+'_optical_to_base'][0,3]=.1;row[eye+'_optical_to_base'][2,3]=.1
    with pytest.raises(ValueError,match='rig'):adapter().validate_capture(row)


def test_stale_image_and_empty_image_rejected():
    row=capture();row['left_render_time_s']=.9
    with pytest.raises(ValueError,match='time'):adapter().validate_capture(row)
    row=capture();row['right_rgb']=np.zeros((0,0,3),np.uint8)
    with pytest.raises(ValueError,match='image'):adapter().validate_capture(row)


def test_mask_version_rejected_before_loading():
    with pytest.raises(ValueError,match='mask'):adapter().validate_mask_identity({'camera_rig':{'rig_id':'old'}})
    assert adapter().validate_mask_identity({'camera_rig':{'rig_id':'ball_stereo_front020_up005_v2'}}) is True
