import numpy as np
import pytest
from neutral_bank_visibility_probe import NeutralBankVisibilityProbe

def call(p,t,frame,count=0,ready=True,action=None):
    return p.choose(t,frame,t,count,ready,np.ones(10,dtype=np.float32) if action is None else action)

def test_distinct_source_updates_and_repeated_current_epoch():
    p=NeutralBankVisibilityProbe();a=call(p,20.,200)
    np.testing.assert_array_equal(a,np.ones(10))
    np.testing.assert_array_equal(p.choose(20.1,200,20.,0,True,np.ones(10)),np.ones(10))
    np.testing.assert_array_equal(call(p,20.5,205),np.zeros(10))

def test_good_update_resets_deficiency_and_before_handoff_equivalence():
    p=NeutralBankVisibilityProbe()
    np.testing.assert_array_equal(call(p,17.,170,ready=False),np.ones(10))
    call(p,18.,180);call(p,18.5,185,100)
    np.testing.assert_array_equal(call(p,19.,190),np.ones(10))
    np.testing.assert_array_equal(call(p,19.5,195),np.zeros(10))

def test_exact_four_second_exit_no_retrigger_no_input_mutation():
    p=NeutralBankVisibilityProbe();original=np.arange(10,dtype=np.float32)
    call(p,18.,180,action=original);call(p,18.5,185,action=original)
    np.testing.assert_array_equal(call(p,22.,220,action=original),np.zeros(10))
    np.testing.assert_array_equal(call(p,22.5,225,action=original),original)
    np.testing.assert_array_equal(call(p,23.,230,action=original),original)
    np.testing.assert_array_equal(original,np.arange(10))

def test_future_source_rejected_before_action_payload_and_rewind_rejected():
    class Trap:
        def __array__(self,*args,**kwargs):raise AssertionError('future action read')
    p=NeutralBankVisibilityProbe()
    with pytest.raises(ValueError,match='clock'):p.choose(1.,10,1.1,0,True,Trap())
    call(p,1.,10)
    with pytest.raises(ValueError,match='clock'):call(p,.9,9)

def test_current_source_age_rejected_and_fractional_frame_count_rejected():
    p=NeutralBankVisibilityProbe()
    with pytest.raises(ValueError):p.choose(1.,1,.4,0,True,np.ones(10))
    with pytest.raises(ValueError):call(p,1.,1.5)
    with pytest.raises(ValueError):call(p,1.,10,.5)
