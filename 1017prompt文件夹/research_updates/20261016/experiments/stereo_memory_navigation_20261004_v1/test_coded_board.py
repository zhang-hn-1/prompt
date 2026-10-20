"""Breaks caught: rotated IDs, border/mask impostors, and wrong USD coordinate signs."""
import numpy as np
from coded_board import decode_patch,detect_tags,world_corners,project_points
CODE=np.array([[1,0,0,1,0],[1,1,0,0,1],[0,1,1,0,1],[1,0,1,1,0],[0,0,1,0,0]],np.uint8)
def tile():
    a=np.zeros((7,7),np.uint8);a[1:6,1:6]=CODE
    return np.repeat(np.repeat(a*255,20,axis=0),20,axis=1)
def test_rotated_code_restores_canonical_corner_order():
    r=decode_patch(np.rot90(tile()),np.array([CODE]))
    assert r['id']==0 and r['rotate_ccw']==3 and r['hamming']==0

def test_border_impostor_rejected():
    assert decode_patch(np.full((140,140),255,np.uint8),np.array([CODE])) is None

def test_masked_tag_is_missing_not_decoded():
    image=np.full((180,180,3),255,np.uint8);image[20:160,20:160]=tile()[...,None]
    mask=np.zeros((180,180),bool)
    tags=detect_tags(image,mask,np.array([CODE]));assert len(tags)==1
    np.testing.assert_allclose(tags[0]['corners'],[[19.5,19.5],[159.5,19.5],[159.5,159.5],[19.5,159.5]],atol=1.)
    mask[65,65]=True
    assert detect_tags(image,mask,np.array([CODE]))==[]

def test_world_corners_use_u_y_v_z_not_xy_plane():
    np.testing.assert_allclose(world_corners([2,3,4],.28),[[2,2.86,4.14],[2,3.14,4.14],[2,3.14,3.86],[2,2.86,3.86]],atol=1e-12)

def test_column_usd_optical_projection_translation():
    pose=np.diag([1.,-1.,-1.,1.]);pose[0,3]=1.
    xy,depth=project_points(np.array([[1,0,2],[0,.2,2],[1,0,-2]]),pose,100,50,40)
    np.testing.assert_allclose(xy[:2],[[50,40],[0,50]],atol=1e-12)
    np.testing.assert_equal(depth,[2,2,-2])

def test_rotated_detected_corners_still_name_same_world_corner():
    image=np.full((180,180,3),255,np.uint8);image[20:160,20:160]=np.rot90(tile())[...,None]
    r=detect_tags(image,np.zeros((180,180),bool),np.array([CODE]))
    assert len(r)==1
    np.testing.assert_allclose(r[0]['corners'],[[19.5,159.5],[19.5,19.5],[159.5,19.5],[159.5,159.5]],atol=1.)

def test_authored_front_face_decodes_without_truth_or_mirror_search():
    # Native front view: +world Z is image right, +world Y is image up.
    # The renderer must author canonical columns along -Y and rows along -Z.
    from coded_board import tag_cell_corners
    image=np.full((180,180,3),255,np.uint8)
    bitmap=np.zeros((7,7),np.uint8);bitmap[1:6,1:6]=CODE
    import cv2
    for row in range(7):
        for col in range(7):
            p=tag_cell_corners([2,0,0],row,col)
            xy=np.c_[90+500*p[:,2],90-500*p[:,1]]
            cv2.fillConvexPoly(image,np.rint(xy).astype(int),(int(bitmap[row,col])*255,)*3)
    tags=detect_tags(image,np.zeros((180,180),bool),np.array([CODE]))
    assert len(tags)==1 and tags[0]['id']==0
    # Canonical TL of code lies world Y+.14,Z+.14 -> image(160,20).
    np.testing.assert_allclose(tags[0]['corners'][0],[160,20],atol=1.5)
