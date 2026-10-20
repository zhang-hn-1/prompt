import numpy as np

def test_exact_one_amplitude_transition_and_same_aux_no_resets():
 from straight_pacing_schedule_v1 import schedule
 a=schedule();assert a.shape==(300,10)
 assert np.all(a[:,1::2]==0)
 assert np.all(a[:60,::2]==0) and np.all(a[60:120,::2]==.5) and np.all(a[120:220,::2]==1.) and np.all(a[220:,::2]==0)
 assert np.all(a[:,::2]==a[:,0,None])
