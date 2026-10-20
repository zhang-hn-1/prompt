import numpy as np,pytest

def source(t=2.,world=((0.,0.,.5),)):
 return dict(time_s=t,frame=round(t*10),frame_width=640,world=np.asarray(world,float).reshape(-1,3),query_pixel_id=np.array([0]*len(world),dtype=int),partner_pixel_id=np.array([1]*len(world),dtype=int))
def masks():
 q={'left':np.zeros((480,640),bool),'right':np.zeros((480,640),bool)};q['left'][0,0]=True;q['right'][0,1]=True;return q

def test_same_exposure_both_eye_metric_support_required_and_far_core_rejected():
 from metric_core_support import MetricCoreSupport
 m=MetricCoreSupport();m.add(source(),masks(),2.)
 keep,proof=m.screen(np.array([[.1,0,.5],[.4,0,.5]]),[20],2.)
 np.testing.assert_equal(keep,[True,False]);assert proof[0]['source_frame']==20 and proof[0]['query_pixel_id']==0 and proof[0]['partner_pixel_id']==1
 assert proof[0]['distance_m']==pytest.approx(.1)
 m2=MetricCoreSupport();cl=masks();cl['right'][:]=False;m2.add(source(),cl,2.)
 assert not m2.screen(np.array([[0,0,.5]]),[20],2.)[0].any()

def test_future_or_outside_window_support_cannot_be_used():
 from metric_core_support import MetricCoreSupport
 m=MetricCoreSupport()
 with pytest.raises(ValueError):m.add({'time_s':3.},masks(),2.)
 m.add(source(),masks(),2.)
 assert not m.screen(np.array([[0,0,.5]]),[25],2.5)[0].any()
 with pytest.raises(ValueError):m.screen(np.array([[0,0,.5]]),[20],1.5)
 with pytest.raises(ValueError):m.add(source(2.5),masks(),float('nan'))

def test_empty_source_does_not_invent_metric_endpoints_and_records_are_immutable():
 from metric_core_support import MetricCoreSupport
 m=MetricCoreSupport();q=source();m.add(q,masks(),2.);q['world'][:]=999
 assert m.screen(np.array([[0,0,.5]]),[20],2.)[0].any()
 m2=MetricCoreSupport();m2.add(source(world=()),masks(),2.)
 assert not m2.screen(np.array([[0,0,.5]]),[20],2.)[0].any()
