import pytest
import reserve_planner as p

def test_cheaper_arrival_cannot_rewrite_existing_child_parent():
    cls=getattr(p,'ArrivalStore',None)
    assert cls is not None,'missing immutable arrival storage'
    store=cls();key=(0,0,0,'forward');childkey=(1,0,0,'forward')
    old=store.offer(key,(0.,0.,0.),5.,None,None)
    child=store.offer(childkey,(.3,0.,0.),6.,old,(.5,0.))
    new=store.offer(key,(.01,0.,0.),3.,None,None)
    assert new!=old and store.best[key]==new
    assert store.records[child].parent_id==old
    assert store.records[old].pose==(0.,0.,0.)
    assert store.records[new].pose==(.01,0.,0.)
    with pytest.raises((AttributeError,TypeError)):
        store.records[old].pose=(10.,0.,0.)

def test_more_expensive_arrival_does_not_replace_best():
    cls=getattr(p,'ArrivalStore',None);assert cls is not None
    store=cls();key=(0,0,0,'reverse')
    initial=store.offer(key,(0.,0.,0.),2.,None,None)
    assert store.offer(key,(0.,0.,0.),3.,None,None) is None
    assert store.best[key]==initial
