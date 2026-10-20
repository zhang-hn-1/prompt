import numpy as np,pytest
from bank_hold_request_program import BankHoldRequestProgram

def test_given_program_only_current_known_mode_and_parent_main():
 commands=np.zeros((3,10),np.float32);commands[2]=np.tile([.5,.2],5);modes=['tracking','GIVEN_PROGRAM_GOAL_PAUSE','tracking'];p=BankHoldRequestProgram(commands,modes,'C')
 np.testing.assert_array_equal(p.act(0,.2,0.,0.),commands[0]);a=p.act(1,.2,.1,.1).reshape(5,2);np.testing.assert_allclose(a[:,1],.2/.75);assert (a[:,0]==0).all();np.testing.assert_array_equal(p.act(2,.2,.2,.2),commands[2])
 with pytest.raises(ValueError):p.act(2,.2,.2,.2)

@pytest.mark.parametrize('arm',['B0','C'])
def test_baseline_exact_and_future_program_poison_does_not_affect_prefix(arm):
 x=np.zeros((2,10),np.float32);y=x.copy();y[1]=np.nan
 a=BankHoldRequestProgram(x,['GIVEN_PROGRAM_GOAL_PAUSE','tracking'],arm);b=BankHoldRequestProgram(y,['GIVEN_PROGRAM_GOAL_PAUSE',None],arm)
 np.testing.assert_array_equal(a.act(0,.2,0.,0.),b.act(0,.2,0.,0.))
