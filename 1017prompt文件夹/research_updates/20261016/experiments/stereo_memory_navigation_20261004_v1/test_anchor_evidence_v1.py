import numpy as np,pytest
from anchor_evidence_v1 import qualify

def args():
 im=np.ones((8,8,3),np.uint8);mask=np.zeros((8,8),bool)
 return [im.copy(),im.copy(),[3.,3.],[3.,3.],mask.copy(),mask.copy(),1.,.5]
def test_nonblack_pair_remains_UNKNOWN_not_measured():
 r=qualify(*args());assert r['eligible_anchor'] and not r['measured_geometry'] and not r['certified_free']
def test_black_query_not_free_or_absent():
 a=args();a[0][3,3]=0;r=qualify(*a);assert not r['eligible_anchor'] and r['query_status']=='black_RGB_unresolved' and r['geometry_role']=='UNKNOWN'
def test_black_source_invalidates_pair_without_changing_original_data():
 a=args();a[1][3,3]=0;before=a[1].copy();r=qualify(*a);assert not r['eligible_anchor'];np.testing.assert_array_equal(a[1],before)
def test_current_CAD_occlusion_not_obstacle():
 a=args();a[4][3,3]=True;r=qualify(*a);assert not r['eligible_anchor'] and r['query_status']=='CAD_occluded'
def test_future_source_rejected_before_payload():
 with pytest.raises(ValueError,match='past'):qualify(None,None,None,None,None,None,1.,2.)
def test_outside_pixel_and_nonuint8_contract():
 a=args();a[2]=[9.,3.];assert qualify(*a)['query_status']=='out_of_image'
 a=args();a[0]=a[0].astype(float)
 with pytest.raises(ValueError,match='RGB'):qualify(*a)
