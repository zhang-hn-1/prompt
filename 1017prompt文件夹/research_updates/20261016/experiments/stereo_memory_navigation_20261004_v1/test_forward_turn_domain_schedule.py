import numpy as np
from forward_turn_domain_schedule import schedule,windows

def test_complete_sequence_has_only_one_forward_domain_and_no_internal_reset():
 a=schedule().reshape(720,5,2);np.testing.assert_equal(a[:60],0);np.testing.assert_equal(a[660:],0);np.testing.assert_equal(a[60:660,:,0],.5);np.testing.assert_equal(a[60:260,:,1],0);np.testing.assert_equal(a[260:460,:,1],np.float32(np.pi/6));np.testing.assert_equal(a[460:660,:,1],np.float32(-np.pi/6));assert np.array_equal(a[:,0],a[:,-1]);a[:]=99;assert schedule()[0,0]==0

def test_three_frozen_windows_stay_in_same_constant_command_without_overlap():
 a=schedule().reshape(720,5,2)
 for lo,hi,aux in windows():
  rows=a[int(lo*10):int(hi*10)+1];assert rows.shape[0]==46;np.testing.assert_equal(rows[:,:,0],.5);np.testing.assert_equal(rows[:,:,1],np.float32(aux))
