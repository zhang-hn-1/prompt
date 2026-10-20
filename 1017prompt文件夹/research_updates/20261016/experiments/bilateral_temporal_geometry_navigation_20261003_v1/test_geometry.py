import numpy as np
import cv2
from geometry import MultiViewGeometry, camera_pose, project, recover_depth


def plane_image(center,rotation,plane_x=3.):
    rng=np.random.default_rng(42)
    texture=cv2.GaussianBlur(rng.integers(0,256,(900,900),dtype=np.uint8),(3,3),0)
    yy,xx=np.mgrid[:480,:640]
    rays=np.stack(((xx-319.5)/(16.45/18*640),(yy-239.5)/(16.45/18*640),np.ones_like(xx)),axis=-1)@rotation.T
    depth=(plane_x-center[0])/rays[:,:,0]
    world=center+rays*depth[:,:,None]
    image=cv2.remap(texture,(450+world[:,:,1]*100).astype('float32'),
                    (450+world[:,:,2]*100).astype('float32'),cv2.INTER_LINEAR)
    return np.repeat(image[:,:,None],3,axis=2)


def test_geometry_reconstructs_textured_plane_without_depth_labels():
    q=np.array([1.,0,0,0]);current=camera_pose(np.array([0.,0.]),q,'left')
    past=[camera_pose(np.array([-.3,0.]),q,'left'),camera_pose(np.array([-.6,0.]),q,'left')]
    yy,xx=np.mgrid[80:400:20,100:540:20];uv=np.column_stack((xx.ravel(),yy.ravel()))
    out=recover_depth(plane_image(*current),current,
                      [(plane_image(*p),p) for p in past],uv=uv)
    assert len(out['depth_m'])>=30
    assert np.percentile(abs(out['depth_m']-2.9),90)<.35


def test_rotation_only_is_unknown_not_invented_depth():
    m=MultiViewGeometry();rgb=np.zeros((480,640,3),dtype=np.uint8)
    for i in range(6):
        out=m.update(rgb,rgb,np.zeros(2),np.array([1.,0,0,0]),i*.2)
    assert len(out['left']['points_world'])==len(out['right']['points_world'])==0


def test_two_eye_inputs_are_independent_and_causal():
    a=MultiViewGeometry();b=MultiViewGeometry();q=np.array([1.,0,0,0])
    outputs=[]
    for i in range(5):
        xy=np.array([-.6+i*.15,0.]);l=plane_image(*camera_pose(xy,q,'left'))
        r=plane_image(*camera_pose(xy,q,'right'))
        x=a.update(l,r,xy,q,i*.2);y=b.update(l,np.zeros_like(r),xy,q,i*.2)
        assert np.array_equal(x['left']['points_world'],y['left']['points_world'])
        outputs.append(x)
    assert len(outputs[-1]['right']['points_world'])>0
    assert len(y['right']['points_world'])==0
    prefix=outputs[-1]['left']['points_world'].copy()
    a.update(np.zeros_like(l),np.zeros_like(r),xy,q,1.2)
    assert np.array_equal(outputs[-1]['left']['points_world'],prefix)


def test_camera_height_approximation_and_rigid_offsets():
    q=np.array([1.,0,0,0]);a,rotation=camera_pose(np.zeros(2),q,'left');b,_=camera_pose(np.zeros(2),q,'right')
    assert np.allclose(a,[.1,.3,.5]);assert np.allclose(a-b,[0,.6,0])
    assert np.allclose(rotation[:,2],[1,0,0])
