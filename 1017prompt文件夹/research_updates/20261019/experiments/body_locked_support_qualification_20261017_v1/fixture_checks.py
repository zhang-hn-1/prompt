"""Small hand-derived causal/coordinate fixtures; no archived data/model/source."""
import numpy as np
from selector import MotionCompatibility,joint_actions

def run():
    # Catches identity-only return: a translated sensor-fixed point must be flagged.
    c=MotionCompatibility();I=np.eye(4);p=np.array([[0.,0.,1.]])
    assert not c.update(0,0.,0.,I,[p])['point_witness'].any()
    T=I.copy();T[0,3]=.3
    r=c.update(1,.5,.5,T,[np.array([[.3,0.,1.]])])
    assert r['anchor_frame']==0 and r['point_witness'].tolist()==[True]
    assert abs(r['distance_world'][0]-.3)<1e-12 and abs(r['distance_body'][0])<1e-12
    # Catches swapped coordinates: world-static point must retain despite local drift.
    c=MotionCompatibility();c.update(0,0.,0.,I,[p]);r=c.update(1,.5,.5,T,[p])
    assert r['point_witness'].tolist()==[False] and r['distance_world'][0]==0 and abs(r['distance_body'][0]-.3)<1e-12
    # Catches missing baseline/empty/history guards.
    c=MotionCompatibility();c.update(0,0.,0.,I,[p]);small=I.copy();small[0,3]=.1
    assert c.update(1,.5,.5,small,[np.array([[.1,0.,1.]])])['anchor_frame']==-1
    assert c.update(2,11.,11.,T,[np.array([[.3,0.,1.]])])['anchor_frame']==-1
    c=MotionCompatibility();c.update(0,0.,0.,I,[np.empty((0,3))]);r=c.update(1,.5,.5,T,[p]);assert not r['point_witness'].any() and np.isnan(r['distance_body']).all()
    # Catches scene-new points being mistaken for sensor-lock.
    c=MotionCompatibility();c.update(0,0.,0.,I,[p]);r=c.update(1,.5,.5,T,[np.array([[2.,0.,1.]])]);assert not r['point_witness'].any()
    # One unsupported point protects a joint cell, regardless of point/view counts.
    assert joint_actions(np.array([0,0,1]),np.array([True,False,True]),2).tolist()==[0.,1.]
    # Initial gauge cannot change a geometrical decision.
    G=np.array([[0.,-1.,0.,7.],[1.,0.,0.,-2.],[0.,0.,1.,.6],[0.,0.,0.,1.]])
    transform=lambda x:x@G[:3,:3].T+G[:3,3]
    c=MotionCompatibility();c.update(0,0.,0.,G,[transform(p)]);rr=c.update(1,.5,.5,G@T,[transform(np.array([[.3,0.,1.]]))])
    assert rr['point_witness'].tolist()==[True] and abs(rr['distance_world'][0]-.3)<1e-12 and abs(rr['distance_body'][0])<1e-12
    class Poison:
        def __array__(self,*a,**k):raise AssertionError('invalid metadata inspected payload')
        def __iter__(self):raise AssertionError('invalid metadata inspected payload')
    for frame,t,now in [(1,.5,.5),(2,2.,1.)]:
        before=(c.frame,c.source_time,len(c.history))
        try:c.update(frame,t,now,Poison(),Poison())
        except ValueError:pass
        else:raise AssertionError('duplicate/future accepted')
        assert before==(c.frame,c.source_time,len(c.history))
    return dict(sensor_lock_flagged=True,world_static_kept=True,baseline_empty_and_expiry_HOLD=True,new_surface_HOLD=True,joint_all_required=True,gauge_invariance=True,duplicate_future_before_payload=True)
if __name__=='__main__':print(run())
