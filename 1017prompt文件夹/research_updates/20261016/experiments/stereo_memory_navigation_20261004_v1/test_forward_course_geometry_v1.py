import numpy as np
from forward_course_geometry_v1 import select_forward_geometry

def test_current_course_changes_only_curvature():
 t=np.arange(5)*.1;v=np.tile([.2,.2],(5,1));m=np.ones(5)*.5
 k,meta=select_forward_geometry(2.,[1.,0.],now=.4,times=t,velocity=v,main_velocity=m,active='forward',pending=None,braking='tracking',reference_direction='forward')
 assert np.isclose(k,-np.sqrt(2)) and meta['used']
 assert meta['parent_curvature']==2.

def test_reverse_low_speed_fallback_and_future_rejected():
 kwargs=dict(now=.4,times=np.arange(5)*.1,velocity=np.tile([.2,0.],(5,1)),main_velocity=np.ones(5)*.5,active='forward',pending=None,braking='tracking',reference_direction='forward')
 for update in [dict(active='reverse'),dict(pending='reverse'),dict(braking='direction_change_braking'),dict(velocity=np.zeros((5,2)))]:
  k,meta=select_forward_geometry(1.,[1.,1.],**(kwargs|update));assert k==1. and not meta['used']
 import pytest
 with pytest.raises(ValueError):select_forward_geometry(1.,[1.,1.],**(kwargs|dict(times=np.arange(5)*.1+.1)))
