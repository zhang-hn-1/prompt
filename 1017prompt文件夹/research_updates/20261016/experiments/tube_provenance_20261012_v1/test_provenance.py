import pytest
from provenance import snapshot_digest, check_order

def trace():
 return dict(sequence_id='s', snapshot_sequence='s', observation_times=(.8,1.), anchor_time=1., snapshot_bytes=b'past-current', snapshot_sha=snapshot_digest('s',(.8,1.),b'past-current'), commit_event=2, observation_events=(0,1), consume_events=tuple(range(3,28)), future_observation_events=(28,29))
def test_valid():assert check_order(**trace())['eligible']
def test_tamper():
 d=trace();d['snapshot_bytes']=b'changed';assert check_order(**d)['reason']=='snapshot_hash_mismatch'
def test_cross_sequence():
 d=trace();d['snapshot_sequence']='other';assert check_order(**d)['reason']=='sequence_mismatch'
def test_future_input():
 d=trace();d['observation_times']=(.8,1.02);d['snapshot_sha']=snapshot_digest('s',d['observation_times'],d['snapshot_bytes']);assert check_order(**d)['reason']=='future_or_missing_anchor'
def test_late_commit():
 d=trace();d['commit_event']=28;assert check_order(**d)['reason']=='event_order_mismatch'
def test_partial():
 d=trace();d['consume_events']=d['consume_events'][:-1];assert check_order(**d)['reason']=='consumption_count_mismatch'
def test_interleaved_future():
 d=trace();d['consume_events']=tuple(range(3,53,2));d['future_observation_events']=(4,54);assert check_order(**d)['eligible']
def test_identity_binds_times():assert snapshot_digest('s',(.8,1.),b'x')!=snapshot_digest('s',(.7,1.),b'x')

def test_future_before_commit():
 d=trace();d["future_observation_events"]=(1,29);assert check_order(**d)["reason"]=="event_order_mismatch"
