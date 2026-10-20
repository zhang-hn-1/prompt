import numpy as np
from correspondence import validate_correspondence

def images_and_disparities():
    random=np.random.default_rng(76);g=random.integers(0,256,(480,640),dtype=np.uint8)
    other=np.zeros_like(g);other[:-80]=g[80:]
    left=np.repeat(g[:,:,None],3,axis=2);right=np.repeat(other[:,:,None],3,axis=2)
    disparity=np.full((480,640),80.,np.float32);masks={e:np.zeros((480,640),bool) for e in ('left','right')}
    return left,right,disparity,masks

def test_corresponding_real_texture_recovers_same_surface():
    left,right,d,masks=images_and_disparities()
    valid=validate_correspondence(left,right,d,d,masks,anchor='left')
    assert valid[200,200]

def test_padding_cannot_be_used_as_an_observed_target_pixel():
    left,right,d,masks=images_and_disparities()
    valid=validate_correspondence(left,right,d,d,masks,anchor='left')
    assert not valid[20,200]

def test_correspondence_hidden_by_other_camera_body_is_unknown():
    left,right,d,masks=images_and_disparities();masks['right'][118:124,198:204]=True
    valid=validate_correspondence(left,right,d,d,masks,anchor='left')
    assert not valid[200,200]

def test_asymmetric_disparities_do_not_create_a_phantom_obstacle():
    left,right,d,masks=images_and_disparities();other=d.copy();other[120,200]=25
    valid=validate_correspondence(left,right,d,other,masks,anchor='left')
    assert not valid[200,200]

def test_textureless_region_is_not_a_depth_measurement():
    left,right,d,masks=images_and_disparities();left[:]=120;right[:]=120
    assert not validate_correspondence(left,right,d,d,masks,anchor='left').any()
