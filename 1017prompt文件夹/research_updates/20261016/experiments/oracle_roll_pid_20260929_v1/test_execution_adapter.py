"""Execution chain regression using production smoother, no Isaac or future logs."""
import math
import numpy as np
from roll_pid import FeedbackSmoother,BankPID
from scripts.action_smoothing import ActionSmoother

class PoseSource:
    def __init__(self):self.angle=0;self.calls=0
    def get_world_pose(self):
        self.calls+=1
        return np.zeros(3),np.array([math.cos(self.angle/2),math.sin(self.angle/2),0,0])

def test_no_feedback_reproduces_real_smoother_and_main_is_always_preserved():
    lo=[-2,-math.pi/6];hi=[2,math.pi/6];pose=PoseSource();rows=[]
    baseline=ActionSmoother(.02,max_auxiliary_velocity_rad_s=.3)
    noop=FeedbackSmoother(ActionSmoother(.02,max_auxiliary_velocity_rad_s=.3),pose,BankPID(0,0,0),rows)
    active=FeedbackSmoother(ActionSmoother(.02,max_auxiliary_velocity_rad_s=.3),pose,BankPID(),[])
    for i in range(400):
        u=np.array([1.5 if i<200 else -1.5,.4 if i<100 else -.4],dtype=np.float32)
        pose.angle=.1*math.sin(i*.02)
        kw=dict(auxiliary_position_rad=.12,auxiliary_velocity_rad_s=.4 if i%7==0 else .1)
        b=baseline.step(u,lo,hi,**kw);n=noop.step(u,lo,hi,**kw);a=active.step(u,lo,hi,**kw)
        np.testing.assert_array_equal(b,n)
        assert a[0]==b[0]
        assert abs(float(a[1]))<=math.pi/6+1e-7
    assert pose.calls==800
    assert len(rows)==400
    assert any(r['supervisor_active'] for r in rows)

def test_prefix_is_independent_of_future_state_sequence():
    def run(future):
        p=BankPID();out=[]
        for bank in [.01,.02,.03]+future:out.append(p.step([.5,.2],bank))
        return np.asarray(out)
    np.testing.assert_array_equal(run([.4,.5])[:3],run([-.4,-.5])[:3])
