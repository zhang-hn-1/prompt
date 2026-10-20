import numpy as np,pytest
from joint_pixel_screen_v2 import screen

def poses():
 a=np.diag([1.,-1.,-1.,1.]);b=a.copy();b[0,3]=.3;return a,b

def test_separate_axis_radius_not_joint_corner_bound():
 a,b=poses();k=np.diag([100.,100.,1.]);r=screen([0,0],[-10,0],a,b,k,1.,.5)
 assert r['joint_max_displacement_m']>r['separate_axis_max_displacement_m']*1.5
 assert not r['all_joint_corners_in_separate_radius'] and not r['certified_bound']
def test_future_source_rejected_before_invalid_payload():
 with pytest.raises(ValueError,match='past'):screen(object(),object(),None,None,None,1.,2.)
def test_same_source_clock_rejected():
 with pytest.raises(ValueError,match='past'):screen(None,None,None,None,None,1.,1.)

def test_result_JSON_serializable():
 import json
 a,b=poses();json.dumps(screen([0,0],[-10,0],a,b,np.diag([100.,100.,1.]),1.,.5))
