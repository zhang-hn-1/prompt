import numpy as np
import pytest
from partial_surface_memory import PartialSurfaceMemory

def packet(t=0.):
    return dict(frame=0,time_s=t,world=np.array([[1.,2.,.5],[2.,3.,.6]]),sigma_m=np.array([.1,.1]),query_pixel_id=np.array([0,1]),partner_pixel_id=np.array([1,0]))

def test_bilateral_positive_only_missing_stays_unknown():
    m=PartialSurfaceMemory();p=packet();a=m.update(p,np.array([True,True]),np.array([False,True]),0.,{'source':'abc'})
    assert len(a['world'])==1
    assert a['query_pixel_id'].tolist()==[0]
    assert a['certified_free'] is False and a['unseen_volume_observed'] is False
    assert m.update(dict(p,frame=1,time_s=.1),np.array([False,False]),np.array([True,True]),.1,{})['world'].shape==(0,3)
    assert sum(len(v['world']) for v in m.at(.1))==1

def test_past_memory_copies_and_does_not_refresh_acquisition():
    m=PartialSurfaceMemory();p=packet();a=m.update(p,np.ones(2,bool),np.ones(2,bool),0.,{'source':'abc'})
    p['world'][:]=99;a['world'][:]=88
    assert m.at(1.)[0]['acquisition_time_s']==0.
    assert m.at(1.)[0]['world'][0,0]==1.
    q=m.at(1.);q[0]['world'][:]=77
    assert m.at(1.)[0]['world'][0,0]==1.

def test_future_packet_rejected_before_payload_and_old_epoch_rejected():
    m=PartialSurfaceMemory()
    with pytest.raises(ValueError):m.update({'time_s':2.},None,None,1.,{})
    m.update(packet(1.),np.ones(2,bool),np.ones(2,bool),1.,{})
    with pytest.raises(ValueError):m.update(packet(),np.ones(2,bool),np.ones(2,bool),0.,{})
