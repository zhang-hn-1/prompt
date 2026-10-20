import pytest
from clock_sync import ensure_current_frames

class CameraFixture:
    def __init__(self,frames):self.frames=frames;self.index=0
    def get_current_frame(self):return self.frames[self.index]
    def progress(self):self.index=min(self.index+1,len(self.frames)-1)

def test_delayed_render_result_is_refreshed_before_pose_pairing():
    c=CameraFixture([{'rendering_time':1.18,'rendering_frame':10},{'rendering_time':1.18,'rendering_frame':10},{'rendering_time':1.2,'rendering_frame':11}])
    frames=ensure_current_frames([c],1.2,c.progress)
    assert frames[0]['rendering_time']==1.2
    assert frames[0]['rendering_frame']==11

def test_unavailable_current_image_is_rejected_instead_of_silently_reusing_old_pose():
    c=CameraFixture([{'rendering_time':1.1,'rendering_frame':10}])
    with pytest.raises(RuntimeError):ensure_current_frames([c],1.2,c.progress)

def test_future_image_is_not_allowed_as_a_current_observation():
    c=CameraFixture([{'rendering_time':1.3,'rendering_frame':12}])
    with pytest.raises(RuntimeError):ensure_current_frames([c],1.2,c.progress)
