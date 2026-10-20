import numpy as np

def test_stop_rebound_resets_five_sample_count():
 from straight_pacing_measurement_v1 import stop_qualification
 t=np.arange(9)*.1;v=np.array([[x,0.] for x in [.04,.04,.06,.04,.04,.04,.04,.04,.06]])
 r=stop_qualification(t,v,0.)
 assert np.isclose(r['first_qualified_s'],.7) and r['last_count']==0 and r['rebound_after_qualification']

def test_no_qualification_from_zero_command_alone():
 from straight_pacing_measurement_v1 import stop_qualification
 r=stop_qualification(np.arange(5)*.1,np.full((5,2),.1),0.)
 assert r['first_qualified_s'] is None

def test_missing_sample_cannot_be_counted_as_continuous_confirmation():
 import pytest
 from straight_pacing_measurement_v1 import stop_qualification
 with pytest.raises(ValueError):stop_qualification([0,.1,.3,.4,.5],np.zeros((5,2)),0.)
