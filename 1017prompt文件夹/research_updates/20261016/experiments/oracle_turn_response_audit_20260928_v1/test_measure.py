import numpy as np
from measure import geometric_curvature,first_sustained

def test_circle_and_reversed_order():
    t=np.linspace(0,2,1001);xy=2*np.c_[np.cos(t),np.sin(t)]
    assert np.max(abs(geometric_curvature(xy)[25:-25]-.5))<.001
    assert np.max(abs(geometric_curvature(xy[::-1])[25:-25]+.5))<.001

def test_line_and_stationary_invalid():
    x=np.linspace(0,10,1001)
    assert np.nanmax(abs(geometric_curvature(np.c_[x,0*x])))==0
    assert np.isnan(geometric_curvature(np.zeros((200,2)))).all()

def test_sustained_not_spike():
    assert first_sustained([0,2,0,2,2,2],1,1,0)==3
    assert first_sustained([0,-2,0],1,-1,0) is None

def test_reversal_requires_handoff_and_sustained_tracking():
    from measure import reversal
    def r(a,d='forward',speed=.5,source='tracking'):
        return {'active_drive_direction':d,'source':source,'world_xy_speed_m_s':speed,'policy_protocol_auxiliary_target_rad':np.radians(a)}
    rows=[r(25,'reverse')]*3+[r(-15,'reverse')]*3+[r(25)]*3+[r(-15),r(5),r(-15),r(-15),r(-15)]
    assert reversal(rows)==(11,-1)
    assert reversal([r(25,'reverse')]*3+[r(-15,'reverse')]*3) is None
