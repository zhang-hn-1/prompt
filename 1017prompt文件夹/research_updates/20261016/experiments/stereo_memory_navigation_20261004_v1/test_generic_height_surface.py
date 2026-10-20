import numpy as np
import pytest
from generic_height_surface import GenericHeightSurfaceMemory

def packet():
 return dict(world=np.array([[0,0,.05],[0,0,.051],[0,0,1.5],[0,0,1.501]]),sigma_m=np.ones(4),query_pixel_id=np.arange(4),partner_pixel_id=np.arange(4),time_s=1.,frame=10)

def test_height_positive_surfaces_ignore_colour_without_inventing_free():
 m=GenericHeightSurfaceMemory(); a=m.update(packet(),np.zeros((480,640),bool),np.zeros((480,640),bool),1.,{'rgb':'hash'})
 assert a['query_pixel_id'].tolist()==[1,2]
 assert a['certified_free'] is False and a['unseen_volume_observed'] is False
 assert a['flat_floor_height_is_prior'] is True
 a['world'][:]=999
 assert m.at(1.)[0]['world'][0,2]==.051

def test_future_clock_rejected_before_payload():
 class Poison(dict):
  def __getitem__(self,k):
   if k=='world': raise AssertionError('future payload read')
   return super().__getitem__(k)
 with pytest.raises(ValueError): GenericHeightSurfaceMemory().update(Poison(time_s=2.),None,None,1.,{})

def test_nonfinite_world_rejected_not_silently_certified():
 p=packet();p['world'][0,0]=np.nan
 with pytest.raises(ValueError):GenericHeightSurfaceMemory().update(p,None,None,1.,{})
