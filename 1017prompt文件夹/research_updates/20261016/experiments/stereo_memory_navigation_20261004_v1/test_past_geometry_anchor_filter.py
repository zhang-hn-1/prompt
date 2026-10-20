import numpy as np
import pytest
from past_geometry_anchor_filter import PastGeometryAnchorFilter

def packet(t,points,frame=None):
 n=len(points)
 return dict(time_s=t,frame=int(t*10) if frame is None else frame,frame_width=640,world=np.array(points,float).reshape(-1,3),sigma_m=np.ones(n)*.1,query_pixel_id=np.arange(n),partner_pixel_id=np.arange(n)+10)

def test_qualification_seed_and_past_only_agreement():
 f=PastGeometryAnchorFilter()
 assert not f.ingest(packet(0,[[0,0,.5]]),0,False).any()
 assert f.ingest(packet(1,[[0,0,.5]]),1,True).tolist()==[True]
 assert f.ingest(packet(2,[[.05,0,.5],[.2,0,.5],[.29,0,.5],[0,0,0]]),2).tolist()==[True,False,False,False]

def test_replay_does_not_refresh_and_expired_seed_not_recreated():
 f=PastGeometryAnchorFilter();p=packet(0,[[0,0,.5]])
 assert f.ingest(p,0,True).all()
 assert f.ingest(p,9).all()
 assert not f.ingest(packet(10.1,[[0,0,.5]]),10.1,True).any()

def test_future_rejected_before_payload_read():
 class Bomb(dict):
  def __getitem__(self,key):
   if key=='world':raise AssertionError('future payload read')
   return super().__getitem__(key)
 with pytest.raises(ValueError):PastGeometryAnchorFilter().ingest(Bomb(time_s=2),1,True)

def test_changed_identity_is_rejected():
 f=PastGeometryAnchorFilter();f.ingest(packet(0,[[0,0,.5]]),0,True)
 with pytest.raises(ValueError):f.ingest(packet(0,[[.01,0,.5]]),1)

def test_translation_and_same_frame_replay():
 a=PastGeometryAnchorFilter();b=PastGeometryAnchorFilter()
 for t,points in [(0,[[0,0,.5]]),(1,[[.09,0,.5],[.18,0,.5]])]:
  p=packet(t,points);q=packet(t,np.array(points)+[100,-30,0])
  np.testing.assert_array_equal(a.ingest(p,t,t==0),b.ingest(q,t,t==0))
 assert a.ingest(p,1).tolist()==[True,False]
