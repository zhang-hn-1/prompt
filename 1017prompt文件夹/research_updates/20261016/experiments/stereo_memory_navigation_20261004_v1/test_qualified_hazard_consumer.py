import numpy as np
import pytest
from qualified_hazard_consumer import QualifiedHazardConsumer, QualifiedHazardGrid

def packet(t, points):
 n=len(points)
 return dict(time_s=t,frame=int(t*10),frame_width=640,world=np.array(points,float).reshape(-1,3),sigma_m=np.full(n,.02),query_pixel_id=np.arange(n),partner_pixel_id=np.arange(n)+10,pose_sigma_m=0.,source_model_admitted=False,certified_free=False)
def proof(t, seed, accepted):
 h=(seed['world'][:,2]>.05)&(seed['world'][:,2]<=1.5)
 return dict(current_appearance_time_s=t,source_geometry_time_s=seed['time_s'],source_frame=seed['frame'],past_world=seed['world'][h],past_query_id=seed['query_pixel_id'][h],past_partner_id=seed['partner_pixel_id'][h],appearance_corroborated=np.array(accepted,bool),geometry_expired=t-seed['time_s']>10)

def test_prefix_passthrough_then_qualified_hazards_keep_nonhazard():
 c=QualifiedHazardConsumer();s=packet(18,[[0,0,.5],[0,0,0]])
 current,past=c.consume(s,None,18,False);np.testing.assert_array_equal(current['world'],s['world']);assert not len(past['world'])
 c.consume(s,proof(18,s,[False]),18,True)
 n=packet(19,[[.05,0,.5],[.3,0,.5],[0,0,0]])
 current,past=c.consume(n,proof(19,s,[True]),19,False)
 np.testing.assert_array_equal(current['world'],n['world'][[0,2]])
 np.testing.assert_array_equal(past['world'],s['world'][[0]])
 assert past['time_s']==18 and past['frame']==180 and past['query_pixel_id'].tolist()==[0] and past['sigma_m'].tolist()==[.02]
 np.testing.assert_array_equal(n['world'],[[.05,0,.5],[.3,0,.5],[0,0,0]])

def test_expired_old_appearance_does_not_refresh_or_make_free():
 c=QualifiedHazardConsumer();s=packet(18,[[0,0,.5]])
 c.consume(s,proof(18,s,[False]),18,True)
 n=packet(28.1,[[.4,0,.5]])
 current,past=c.consume(n,proof(28.1,s,[False]),28.1,False)
 assert not len(current['world']) and not len(past['world'])
 assert not current['certified_free'] and not past['certified_free']

def test_future_packet_and_future_appearance_rejected_before_payload():
 class Bomb(dict):
  def __getitem__(self,k):
   if k=='world' or k=='past_world':raise AssertionError('future payload read')
   return super().__getitem__(k)
 with pytest.raises(ValueError):QualifiedHazardConsumer().consume(Bomb(time_s=20),None,19,False)
 with pytest.raises(ValueError):QualifiedHazardConsumer().consume(packet(19,[]),Bomb(current_appearance_time_s=20),19,False)

def test_grid_consumes_old_time_and_rejected_hazard_is_unknown():
 c=QualifiedHazardConsumer();s=packet(18,[[0,0,.5]])
 c.consume(s,proof(18,s,[False]),18,True)
 current,past=c.consume(packet(19,[[.4,0,.5]]),proof(19,s,[True]),19,False)
 g=QualifiedHazardGrid(np.array([-1.,-1.]),shape=(30,30),resolution=.1,memory_mode='current')
 g.set_source(current);g.set_memory_source(past);g.update_points([],np.array([0,0,0]),19)
 assert g.labels[10,10]==1 and g.provenance()[10,10]==4 and g.memory.hazard_time[10,10]==18
 assert g.labels[14,10]==-1
