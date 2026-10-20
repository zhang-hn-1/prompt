import numpy as np
import pytest
from shell_link_measurement import copy_link_state,bottom_velocity

def test_named_extraction_is_copied_not_root_position_assumed():
 p=np.array([[[0,0,.4,0,0,0,1],[0,0,.4,0,0,0,1]]],float);v=np.zeros((1,2,6));r=copy_link_state(0,['link1','base_link'],p,v);p[:]=20;assert r['link_names']==['link1','base_link'] and r['poses_xyzw'][0][2]==.4

def test_missing_shell_or_bad_shape_refused():
 with pytest.raises(ValueError):copy_link_state(0,['base_link'],np.zeros((1,1,7)),np.zeros((1,1,6)))
 with pytest.raises(ValueError):copy_link_state(0,['base_link','link1'],np.zeros((1,2,6)),np.zeros((1,2,6)))

def test_nonfinite_and_duplicate_names_refused():
 with pytest.raises(ValueError):copy_link_state(0,['base_link','link1'],np.full((1,2,7),np.nan),np.zeros((1,2,6)))
 with pytest.raises(ValueError):copy_link_state(0,['base_link','link1','link1'],np.zeros((1,3,7)),np.zeros((1,3,6)))

def test_sphere_contact_signed_velocity():
 np.testing.assert_allclose(bottom_velocity([.8,0,0],[0,2,0],.4),[0,0,0],atol=1e-15)
 np.testing.assert_allclose(bottom_velocity([.9,0,0],[0,2,0],.4),[.1,0,0],atol=1e-15)
 np.testing.assert_allclose(bottom_velocity([0,0,0],[0,0,2],.4),[0,0,0],atol=1e-15)
