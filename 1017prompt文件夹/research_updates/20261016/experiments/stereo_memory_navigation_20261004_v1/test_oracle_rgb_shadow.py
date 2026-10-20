import sys,copy
from pathlib import Path
import numpy as np
import pytest
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
import oracle_rgb_shadow as m

def packet(i=0,t=0.):
 return dict(frame=i,time_s=t,left_rgb=np.zeros((64,64,3),np.uint8),right_rgb=np.zeros((64,64,3),np.uint8),left_camera_usd_world=np.eye(4),right_camera_usd_world=np.eye(4))

def test_no_truth_keys_and_copy_isolation():
 # Mutation: forwarding arbitrary runtime fields or aliases would let shadow consume truth/control state.
 p=packet();s=m.sanitize(p);s['left_rgb'][:]=255;s['left_camera_usd_world'][0,3]=10
 assert p['left_rgb'].sum()==0 and p['left_camera_usd_world'][0,3]==0
 with pytest.raises(ValueError):m.sanitize(dict(p,depth=np.ones((64,64))))
 with pytest.raises(ValueError):m.sanitize(dict(p,oracle_reference=[[1,1]]))
 with pytest.raises(ValueError):m.sanitize(packet(t=np.nan))

def test_no_floor_prior_and_atomic_bad_time():
 # Mutation: seeding footprint or accepting a duplicated/past packet changes unknown/support or time.
 s=m.OnlineRGBShadow(dict(left=np.zeros((64,64),bool),right=np.zeros((64,64),bool)))
 a=s.observe(packet());assert a['causal']['points']==[] and a['floor_cells']==0 and a['certified_free'] is False
 with pytest.raises(ValueError):s.observe(packet())
 b=s.observe(packet(1,.1));assert b['source_views']==[[1,'right',.1],[0,'left',0.],[0,'right',0.]] and b['floor_cells']==0

def test_future_packet_skipped_before_payload():
 # Mutation: reading future pixels before filtering causes the poison fixture to throw.
 class Trap(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload')
   return super().__getitem__(k)
 p=[packet(0,0.),packet(1,.1)]
 a=m.replay_shadow(p,.1,masks=dict(left=np.zeros((64,64),bool),right=np.zeros((64,64),bool)))
 b=m.replay_shadow(p+[Trap(time_s=.2)],.1,masks=dict(left=np.zeros((64,64),bool),right=np.zeros((64,64),bool)))
 assert a['causal']==b['causal'] and a['records']==b['records']

def test_cached_reference_literal_direction_and_commands():
 # Mutation: reconstructing the last zero command or reverse direction incorrectly changes tracking.
 poses=np.array([[0,0,0],[1,0,0],[1,1,1.57]])
 plan=m.reference_from_arrays(poses,np.array([1,-1,-1]),np.array([[.5,.2],[-.5,-.2],[0,0]]),2.,3)
 assert plan.waypoints[1].drive_direction=='reverse' and plan.waypoints[-1].main_command_rad_s==0 and plan.path_length_m==2.

def test_real_oracle_shadow_on_off_keeps_actions_and_state():
 # Mutation: letting observation-only shadow alter policy/observation changes the same-input action sequence.
 from benchmark.policies import PolicyObservation,EpisodeContext
 from training.action_protocol import load_action_protocol
 protocol=load_action_protocol(P/'configs/data_contract.yaml')
 a=m.make_oracle(protocol);b=m.make_oracle(protocol)
 plan=m.reference_from_arrays(np.array([[0,0,0],[1,0,0],[2,0,0]]),np.ones(3),np.array([[.5,0],[.5,0],[0,0]]),2.,1)
 ctx=EpisodeContext('literal',True,np.array([[0.,0.],[2.,0.]]),protocol.protocol_id,plan.waypoints);a.reset(ctx);b.reset(ctx)
 s=m.OnlineRGBShadow(dict(left=np.zeros((64,64),bool),right=np.zeros((64,64),bool)))
 rgb=np.zeros((480,640,3),np.uint8)
 for i in range(8):
  obs=PolicyObservation(rgb,rgb,np.zeros((2,17),np.float32),np.zeros((2,10),np.float32),np.array([i*.05,0.]),0.)
  action=a.act(obs,np.array([2-i*.05,0.]));s.observe(packet(i,i*.1));other=b.act(obs,np.array([2-i*.05,0.]))
  np.testing.assert_equal(action,other);assert a.diagnostics==b.diagnostics and a.request_diagnostics==b.request_diagnostics and a.mapping_records==b.mapping_records
