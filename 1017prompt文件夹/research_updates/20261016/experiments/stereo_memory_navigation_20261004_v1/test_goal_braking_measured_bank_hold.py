import numpy as np
from goal_braking_measured_bank_hold import GoalBrakingMeasuredBankHold

def test_latches_current_bank_preserves_main_clears_on_restart():
 p=GoalBrakingMeasuredBankHold();r=np.array([0.,0.]);a=p.request(r,'goal_braking',.2,1.,1.);np.testing.assert_allclose(a,[0.,.2/.75]);b=p.request(r,'goal_braking',-.1,1.1,1.1);np.testing.assert_array_equal(a,b)
 z=np.array([.5,.1]);np.testing.assert_array_equal(p.request(z,'tracking',.2,1.2,1.2),z);np.testing.assert_allclose(p.request(r,'goal_braking',-.15,1.3,1.3),[0.,-.15/.75])

def test_invalid_or_unrepresentable_measurement_falls_through_no_later_latch():
 for bank,t in [(float('nan'),1.),(.5,1.),(.1,1.01),(.1,.7)]:
  p=GoalBrakingMeasuredBankHold();r=np.array([0.,.05]);np.testing.assert_array_equal(p.request(r,'goal_braking',bank,t,1.),r);np.testing.assert_array_equal(p.request(r,'goal_braking',.1,1.1,1.1),r)

def test_input_copy_and_normal_branch_exact():
 p=GoalBrakingMeasuredBankHold();r=np.array([-.3,-.2],np.float32);a=p.request(r,'tracking',.2,1.,1.);a[0]=999;np.testing.assert_array_equal(r,np.array([-.3,-.2],np.float32));assert p.latched_bank is None

def test_nonzero_main_while_goal_braking_resets_independently():
 p=GoalBrakingMeasuredBankHold();p.request(np.array([0.,0.]),'goal_braking',.2,1.,1.)
 p.request(np.array([.5,0.]),'goal_braking',.2,1.1,1.1);assert p.latched_bank is None
 np.testing.assert_allclose(p.request(np.array([0.,0.]),'goal_braking',-.1,1.2,1.2),[0.,-.1/.75])

def test_zero_main_leaving_goal_state_resets_independently():
 p=GoalBrakingMeasuredBankHold();p.request(np.array([0.,0.]),'goal_braking',.2,1.,1.)
 p.request(np.array([0.,0.]),'tracking',.2,1.1,1.1);assert p.latched_bank is None
 np.testing.assert_allclose(p.request(np.array([0.,0.]),'goal_braking',-.1,1.2,1.2),[0.,-.1/.75])
