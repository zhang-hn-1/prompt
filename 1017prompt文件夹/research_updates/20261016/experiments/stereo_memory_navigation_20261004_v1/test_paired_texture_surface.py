import numpy as np
import pytest
from paired_texture_surface import PairedTextureSurfaceMemory

def p():return dict(world=np.array([[0.,0.,.5]]),sigma_m=np.ones(1)*.01,query_pixel_id=np.array([50*640+50]),partner_pixel_id=np.array([40*640+50]),time_s=1.,frame=10)

def test_constant_pair_cannot_supply_positive_risk():
 b=np.zeros((480,640,3),np.uint8);m=PairedTextureSurfaceMemory();r=m.update(p(),b,b,1.,{},np.ones((480,640))*10)
 assert len(r['world'])==0 and len(m.at(1.)[0]['world'])==0 and r['certified_free'] is False

def test_shifted_textured_pair_keeps_original_depth_and_ids():
 rng=np.random.default_rng(2);l=rng.integers(0,256,(480,640,3),dtype=np.uint8);r=np.zeros_like(l);r[:-10]=l[10:];m=PairedTextureSurfaceMemory();a=m.update(p(),l,r,1.,{},np.ones((480,640))*10)
 assert np.array_equal(a['world'],p()['world']) and np.array_equal(a['partner_pixel_id'],p()['partner_pixel_id'])
 a['world'][:]=999;assert m.at(1.)[0]['world'][0,2]==.5

def test_future_packet_rejected_before_rgb_or_point_payload():
 class Poison(dict):
  def __getitem__(self,k):
   if k=='world':raise AssertionError('future physical payload')
   return super().__getitem__(k)
 with pytest.raises(ValueError):PairedTextureSurfaceMemory().update(Poison(time_s=2.),None,None,1.,{},None)

def test_bad_rgb_does_not_commit_a_source_record():
 m=PairedTextureSurfaceMemory()
 with pytest.raises(ValueError):m.update(p(),np.zeros((1,1,3),np.uint8),None,1.,{},np.ones((480,640))*10)
 assert m.at(1.)==[]
