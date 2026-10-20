"""Literal geometry and inherited real methods; no planner/sim/mock."""
import math
import numpy as np
import pytest
from outgoing_lookahead import make_endpoint_oracle,EndpointLookaheadPolicy
from cusp_projection import make_cusp_oracle,CuspProjectionPolicy
from oracle_rgb_shadow import reference_from_arrays
from depth_local_curve import install_local_reference,P
from benchmark.policies import PolicyObservation,EpisodeContext,OraclePathPolicy
from training.action_protocol import load_action_protocol

def protocol():return load_action_protocol(P/'configs/data_contract.yaml')
def tracker(pose,directions,commands,corrected=True):
 t=make_endpoint_oracle(protocol()) if corrected else make_cusp_oracle(protocol())
 install_local_reference(t,reference_from_arrays(pose,directions,commands,3.,1));return t

def obs(xy,yaw=0.,speed=.1):
 rgb=np.zeros((480,640,3),np.uint8);proprio=np.zeros((2,17),np.float32);proprio[-1,0]=speed
 return PolicyObservation(rgb,rgb,proprio,np.zeros((2,10),np.float32),np.asarray(xy,float),yaw)

def pp(t,xy,yaw=0.):
 target=t._lookahead_target(.5);d=target-np.asarray(xy);c,s=math.cos(yaw),math.sin(yaw)
 b=np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1]])
 return 0. if b@b<=1e-9 else 2*b[1]/(b@b)

@pytest.mark.parametrize('sign',[1,-1])
def test_single_opposite_outgoing_edge_includes_endpoint_not_its_start(sign):
 # Break: NEXT direction tested before CURRENT edge drops reverse endpoint.
 t=tracker([[0,0,0],[1,0,0],[.8,.2,0],[2,.2,0]],[sign,-sign,sign,sign],[[.5*sign,0],[-.5*sign,0],[.5*sign,0],[0,0]])
 t._path_index=1;t._active_drive_direction='reverse' if sign>0 else 'forward'
 np.testing.assert_allclose(t._lookahead_target(.5),[.8,.2],atol=0,rtol=0)
 assert t._tracking_audit['lookahead_index']==2 and t._tracking_audit['lookahead_accumulated_m']==pytest.approx(math.sqrt(.08))
 assert t.path_index==1

@pytest.mark.parametrize('sign',[1,-1])
def test_clamp_at_last_current_edge_never_enter_following_opposite_edge(sign):
 # Break: include endpoint but traverse its opposite outgoing edge.
 t=tracker([[0,0,0],[.3,0,0],[.6,0,0],[.3,.3,0]],[sign,sign,-sign,-sign],[[.5*sign,0],[.5*sign,0],[-.5*sign,0],[0,0]])
 t._path_index=1
 np.testing.assert_allclose(t._lookahead_target(2),[.6,0],atol=0,rtol=0)
 assert t._tracking_audit['lookahead_accumulated_m']==pytest.approx(.3)

@pytest.mark.parametrize('sign,k',[(1,1),(1,-1),(-1,1),(-1,-1)])
def test_ideal_constant_curvature_final_edge_has_hand_derived_body_curvature(sign,k):
 # Break: final edge returns start, producing zero instead of circular PP curvature.
 t=tracker([[0,0,0],[sign*.5,k*.1339745962155614,sign*k*math.pi/6],[sign*.8,0,0]], [sign,-sign,-sign],[[.5*sign,0],[-.5*sign,0],[0,0]])
 assert pp(t,[0,0])==pytest.approx(k,abs=1e-12)

@pytest.mark.parametrize('sign,xy,yaw,want',[(1,[0,.05],0,-.09975062344139651),(-1,[0,.05],0,-.09975062344139651),(1,[0,0],.1,-.1996668332936563),(-1,[0,0],.1,.1996668332936563)])
def test_small_position_and_heading_perturbations_have_signed_hand_derived_response(sign,xy,yaw,want):
 # Break: curvature sign uses motion sign twice or query drops straight endpoint.
 t=tracker([[0,0,0],[sign,0,0],[sign*.8,0,0]],[sign,-sign,-sign],[[.5*sign,0],[-.5*sign,0],[0,0]])
 assert pp(t,xy,yaw)==pytest.approx(want,abs=1e-12)

@pytest.mark.parametrize('sign',[1,-1])
def test_same_direction_full_act_and_diagnostics_remain_exact(sign):
 # Break: correction spills into ordinary geometry/mapping/request behavior.
 args=([[0,0,0],[.3,0,0],[.6,0,0],[1,0,0],[2,0,0]],[sign]*5,[[.5*sign,0]]*4+[[0,0]])
 old,new=tracker(*args,corrected=False),tracker(*args)
 for xy in [[.05,.02],[.35,.02],[.7,0]]:
  np.testing.assert_array_equal(old.act(obs(xy),[4,0]),new.act(obs(xy),[4,0]))
  assert old.diagnostics==new.diagnostics and old.request_diagnostics==new.request_diagnostics and old.tracking_diagnostics==new.tracking_diagnostics
 for t in [old,new]:
  t._path_index=4;np.testing.assert_array_equal(t._lookahead_target(.5),[2,0])

def test_boundary_query_does_not_change_five_low_rebound_or_outer_method_identity():
 # Break: fix changes latch/five-low/speed or action-time reset.
 t=tracker([[0,0,0],[1,0,0],[.8,.2,0],[2,.2,0]],[1,-1,1,1],[[.5,0],[-.5,0],[.5,0],[0,0]])
 t._path_index=1
 for speed,want in [(.04,1),(.04,2),(.06,0),(.04,1),(.04,2),(.04,3),(.04,4)]:
  action=t.act(obs([1,0],speed=speed),[4,0]);np.testing.assert_array_equal(action,np.zeros(10,np.float32));assert t._stopped_cycles==want and t.path_index==1
 action=t.act(obs([1,0],speed=.04),[4,0]);assert action[0]<0 and t._active_drive_direction=='reverse' and t._pending_drive_direction is None and t._stopped_cycles==0
 assert t._tracking_audit['lookahead_index']==2
 assert EndpointLookaheadPolicy.act is CuspProjectionPolicy.act and EndpointLookaheadPolicy.plan_boundary is CuspProjectionPolicy.plan_boundary
 assert t.act.__func__ is OraclePathPolicy.act
 p=EndpointLookaheadPolicy(protocol());p.reset(EpisodeContext('literal',False,None,p.protocol.protocol_id));assert p.tracker._reference is None
