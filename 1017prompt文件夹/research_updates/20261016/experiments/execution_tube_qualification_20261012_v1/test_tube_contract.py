from dataclasses import FrozenInstanceError
import pytest
from tube_contract import Proposal,qualify_label

def proposal():return Proposal(1.,tuple((.3,.1) for _ in range(25)), 'seq-a', 'a'*64)
def label():return dict(sequence_id='seq-a', current_observation_sha256='a'*64, times=tuple(1.+i*.02 for i in range(26)),positions=tuple((i*.01,0.) for i in range(26)),raw_targets=tuple((.3,.1) for _ in range(25)),complete=True)

def test_matching_label():
 assert qualify_label(proposal(),**label())=={'scorable':True,'reason':'exact_completed_program'}

def test_proposal_is_immutable():
 with pytest.raises(FrozenInstanceError):proposal().time_s=2.

def test_mutable_commands_rejected():
 with pytest.raises(ValueError):Proposal(1.,[[.3,.1]]*25, 'seq-a', 'a'*64)

def test_mismatched_program():
 data=label();data['raw_targets']=((.3,.2),)+data['raw_targets'][1:]
 assert qualify_label(proposal(),**data)['reason']=='program_mismatch'

def test_clock_shift():
 data=label();data['times']=tuple(t+.02 for t in data['times'])
 assert qualify_label(proposal(),**data)['reason']=='clock_mismatch'

def test_incomplete_preserved():
 data=label();data['complete']=False;data['times']=data['times'][:10];data['positions']=data['positions'][:10]
 assert qualify_label(proposal(),**data)['reason']=='censored_execution'

def test_nonfinite_label_rejected():
 data=label();data['positions']=((float('nan'),0.),)+data['positions'][1:]
 with pytest.raises(ValueError):qualify_label(proposal(),**data)

def test_missing_samples_preserved():
 data=label();data['raw_targets']=data['raw_targets'][:-1]
 assert qualify_label(proposal(),**data)['reason']=='missing_samples'

def test_raw_float32_encoding_is_canonical():
 import struct
 p=proposal();data=label();data['raw_targets']=tuple(tuple(struct.unpack('f',struct.pack('f',v))[0] for v in row) for row in data['raw_targets'])
 assert qualify_label(p,**data)['scorable']

def test_sequence_mismatch():
 data=label();data["sequence_id"]="seq-b"
 assert qualify_label(proposal(),**data)["reason"]=="sequence_mismatch"

def test_observation_mismatch():
 data=label();data["current_observation_sha256"]="b"*64
 assert qualify_label(proposal(),**data)["reason"]=="observation_mismatch"
