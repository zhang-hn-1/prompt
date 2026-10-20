import copy
import numpy as np
import pytest
from static_seed_belief import StaticSeedBeliefGrid,SafeStartBootstrap

def packet(t=0.,frame=0):
 return dict(world=np.array([[0.,0.,.5],[.1,0.,.5]]),sigma_m=np.zeros(2),query_pixel_id=np.array([12,14]),partner_pixel_id=np.array([9,11]),frame=frame,time_s=t,frame_width=640)

def grid():return StaticSeedBeliefGrid(np.array([-1.,-1.]),shape=(40,40),resolution=.1)
def update(g,p,t):
 g.set_source(p);g.update_points(p['world'],np.array([-.9,-.9,.4]),t)
def test_seed_age_is_immutable_and_belief_not_observation():
 g=grid();p=packet();g.seed(p,0.);update(g,p,0.);assert g.provenance()[10,10]==3
 empty=packet(5.,50);empty['world']=np.empty((0,3));empty['sigma_m']=np.empty(0);empty['query_pixel_id']=empty['partner_pixel_id']=np.empty(0,int)
 update(g,empty,5.);assert g.provenance()[10,10]==4 and g.memory.hazard_time[10,10]==0.
 empty['time_s']=11.;empty['frame']=110;update(g,empty,11.)
 assert g.labels[10,10]==1 and g.provenance()[10,10]==5
 assert not np.isfinite(g.memory.hazard_time[10,10])
 assert g.belief_observation_time[10,10]==0. and g.belief_seed['query_pixel_id'].tolist()==[12,14]
 assert g.memory.floor_time[10,10]==-np.inf and g.diagnostics()['static_belief_is_observation'] is False

def test_same_seed_payload_change_rejected_without_mutation():
 g=grid();p=packet();g.seed(p,0.);bad=copy.deepcopy(p);bad['world'][0,0]=.2
 with pytest.raises(ValueError):g.seed(bad,1.)
 assert np.array_equal(g.belief_seed['world'],p['world'])

def test_future_seed_rejected_before_payload():
 class Poison(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return 2.
 with pytest.raises(ValueError):grid().seed(Poison(),1.)

def test_unseeded_map_is_current_baseline():
 from prior_rgb_navigation import PriorRGBGrid
 a=grid();b=PriorRGBGrid(np.array([-1.,-1.]),shape=(40,40),memory_mode='current')
 for g in [a,b]:update(g,packet(),0.)
 assert np.array_equal(a.labels,b.labels) and np.array_equal(a.provenance(),b.provenance())

def test_belief_expires_at_task_limit():
 g=grid();g.seed(packet(),0.);p=packet(60.,600);update(g,p,60.)
 assert not np.isfinite(g.belief_observation_time).any()

def test_bootstrap_current_confirmation_and_five_low_counts():
 gate=SafeStartBootstrap();a=np.ones(10,np.float32);event=dict(time_s=.5,frame=5,confirmed=True)
 p=packet(.5,5);p['world']=np.tile(p['world'],(60,1))
 for i in range(4):assert np.array_equal(gate.choose(i*.1,0.,p,a,None,i),np.zeros(10))
 assert np.array_equal(gate.choose(.5,0.,p,a,event,5),a) and gate.handoff_time_s==.5

def test_bootstrap_rebound_and_deadline_do_not_force_release():
 gate=SafeStartBootstrap();p=packet();a=np.ones(10)
 for i in range(4):gate.choose(i*.1,0.,p,a,None,i)
 gate.choose(.4,.05,p,a,None,4);assert gate.low_count==0
 assert not gate.choose(8.,0.,p,a,None,80).any() and gate.blocked
 p=packet(8.5,85);p['world']=np.tile(p['world'],(60,1))
 assert not gate.choose(8.5,0.,p,a,dict(time_s=8.5,frame=85,confirmed=True),85).any()

def test_future_event_rejected_before_confirmation():
 class Poison(dict):
  def __getitem__(self,k):
   if k not in ('time_s','frame'):raise AssertionError('future event payload read')
   return 2. if k=='time_s' else 20
 with pytest.raises(ValueError):SafeStartBootstrap().choose(1.,0.,packet(),np.ones(10),Poison(),10)


def test_seed_rejects_future_frame_and_changed_frame_width():
 g=grid();p=packet();p['frame']=1000
 with pytest.raises(ValueError):g.seed(p,0.)
 p=packet();g.seed(p,0.);p['frame_width']=639
 with pytest.raises(ValueError):g.seed(p,1.)

def test_measured_floor_is_not_absence_of_above_floor_static_hazard():
 g=grid();g.seed(packet(),0.);p=packet(11.,110);p['world'][:,2]=0.;update(g,p,11.)
 assert g.labels[10,10]==1 and g.provenance()[10,10]==5
 assert g.memory.floor_time[10,10]==11. and g.memory.hazard_time[10,10]==-np.inf
 assert g.belief_observation_time[10,10]==0.
