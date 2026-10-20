import numpy as np,pytest
from bilateral_patch_confidence import filter_packet

def data():
 rng=np.random.default_rng(17);l=rng.integers(0,256,(40,40,3),dtype=np.uint8);r=np.zeros_like(l);r[:-6]=l[6:];mask=np.zeros((40,40),bool)
 p=dict(world=np.array([[1.,2.,.5]]),sigma_m=np.array([.01]),query_pixel_id=np.array([20*40+20]),partner_pixel_id=np.array([14*40+20]),time_s=0.,frame=0,frame_width=40)
 return l,r,mask,p

def test_distinctive_true_match_retained_without_payload_change():
 l,r,m,p=data();q,d=filter_packet(p,l,r,m,m,0.)
 assert len(q['world'])==1 and d['retained']==1
 for k in p:np.testing.assert_equal(q[k],p[k])

def test_wrong_correspondence_rejected():
 l,r,m,p=data();p['partner_pixel_id'][0]=10*40+20;q,d=filter_packet(p,l,r,m,m,0.);assert len(q['world'])==0

def test_flat_and_repeated_texture_are_ambiguous():
 l,r,m,p=data();l[:]=100;r[:]=100;q,d=filter_packet(p,l,r,m,m,0.);assert not len(q['world'])
 # Nonflat but vertically repeated bands: NCC alone passes, uniqueness must fail.
 yy,xx=np.indices((40,40));a=((xx%2)*200).astype(np.uint8);l=np.repeat(a[:,:,None],3,2);r=l.copy();q,d=filter_packet(p,l,r,m,m,0.);assert not len(q['world']) and d['non_unique']==1

def test_any_patch_CAD_occlusion_or_border_rejected():
 l,r,m,p=data();bad=m.copy();bad[14,21]=True;q,d=filter_packet(p,l,r,m,bad,0.);assert not len(q['world'])
 p['query_pixel_id'][0]=1;q,d=filter_packet(p,l,r,m,m,0.);assert not len(q['world'])

def test_future_clock_rejected_before_RGB_or_source_payload():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return 1.
 with pytest.raises(ValueError):filter_packet(Bomb(),None,None,None,None,0.)
