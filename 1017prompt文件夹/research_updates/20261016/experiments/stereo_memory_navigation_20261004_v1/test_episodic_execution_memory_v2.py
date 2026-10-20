"""Breaks caught: future clocks, realized-command disguise, heldout retrieval and mutation."""
import importlib
import numpy as np
import pytest

def api():return importlib.import_module('episodic_execution_memory_v2')
def packet(now=1.):
 return dict(times=np.arange(5)*.1+now-.4,proprio=np.zeros((5,17)),past_actions=np.zeros((5,2)),yaw_current=0.,privilege='simulator_state_diagnostic')
def test_missing_then_minted_proposal_has_no_future_command_input():
 m=api();p=m.mint_hold_proposal(.5,.1,1.);assert p.commands.shape==(5,2);assert p.created_at==1.
 with pytest.raises(ValueError):m.proposal_from_record(dict(commands=np.zeros((5,2)),created_at=1.,producer='future_realized_log'))
def test_history_clock_rejects_future_payload_before_features():
 m=api();p=packet();p['times'][-1]=1.1
 with pytest.raises(ValueError):m.features(p,1.)
def test_exact_outcome_retrieval_and_out_of_program_unknown():
 m=api();f=m.features(packet(),1.);p=m.mint_hold_proposal(.5,0.,1.);target=np.array([[.02*i,0.,0.] for i in range(1,6)])
 lib=m.Memory([dict(episode='trainA',frame=10,feature=f,program=p.commands,target=target)],allowed_episodes={'trainA'},forbidden_episodes={'testA'})
 got=lib.query(f,p);assert got['status']=='supported';np.testing.assert_array_equal(got['trajectory'],target)
 assert lib.query(f,m.mint_hold_proposal(-.3,0.,1.))['status']=='unknown'
def test_heldout_library_entry_rejected():
 m=api();entry=dict(episode='testA',frame=10,feature=m.features(packet(),1.),program=np.zeros((5,2)),target=np.zeros((5,3)))
 with pytest.raises(ValueError):m.Memory([entry],allowed_episodes={'trainA'},forbidden_episodes={'testA'})
def test_memory_owns_copies_and_query_output_cannot_corrupt_next_prediction():
 m=api();f=m.features(packet(),1.);p=m.mint_hold_proposal(.5,0.,1.);t=np.ones((5,3));e=dict(episode='trainA',frame=10,feature=f,program=p.commands,target=t)
 lib=m.Memory([e],allowed_episodes={'trainA'},forbidden_episodes={'testA'});t[:]=99
 got=lib.query(f,p);assert got['trajectory'][0,0]==1.;got['trajectory'][:]=55;assert lib.query(f,p)['trajectory'][0,0]==1.
 with pytest.raises(ValueError):p.commands[0,0]=22
 def_attempt=packet();def_attempt['proprio'][0,0]=np.nan
 with pytest.raises(ValueError):m.features(def_attempt,1.)
def test_world_gauge_rotation_and_angle_wrap_do_not_change_matching():
 m=api();a=packet();a['proprio'][:,0]=.3;a['proprio'][:,9]=0
 b=packet();b['yaw_current']=np.pi/2;b['proprio'][:,1]=.3;b['proprio'][:,9]=2*np.pi
 np.testing.assert_allclose(m.features(a,1.),m.features(b,1.),atol=1e-12)
def test_empty_library_and_far_state_return_unknown():
 m=api();f=m.features(packet(),1.);p=m.mint_hold_proposal(.5,0.,1.);empty=m.Memory([],allowed_episodes={'trainA'},forbidden_episodes={'testA'});assert empty.query(f,p)['status']=='unknown'
 lib=m.Memory([dict(episode='trainA',frame=10,feature=f,program=p.commands,target=np.zeros((5,3)))],allowed_episodes={'trainA'},forbidden_episodes={'testA'});assert lib.query(f+1,p)['status']=='unknown'

def test_stale_proposal_and_nonconstant_record_rejected():
 m=api();f=m.features(packet(),1.);lib=m.Memory([],allowed_episodes={'trainA'},forbidden_episodes={'testA'})
 with pytest.raises(ValueError):lib.query(f,m.mint_hold_proposal(.5,0.,.9),now=1.)
 a=np.zeros((5,2));a[4,0]=.5
 with pytest.raises(ValueError):m.proposal_from_record(dict(commands=a,created_at=1.,producer='hold_current_request_v1'))
