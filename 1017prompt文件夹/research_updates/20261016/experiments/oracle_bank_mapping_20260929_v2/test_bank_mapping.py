import math
import numpy as np
import bank_mapping as m

def fixture():
 return {'directions':{d:{'low':{'speed_m_s':.2,'bank_rad':[-.3,0,.3],'curvature_body_1_m':[.6,0,-.6]},'high':{'speed_m_s':.6,'bank_rad':[-.3,0,.3],'curvature_body_1_m':[.9,0,-.9]}} for d in ['forward','reverse']}}

def test_inverse_recovers_bank_for_body_curvature_in_both_directions():
 assert hasattr(m,'BankMap'), 'calibrated inverse missing'
 model=m.BankMap(fixture())
 for sign in [1,-1]:
  assert np.isclose(model.query(sign,.2,.3)['bank_rad'],-.15)
  assert np.isclose(model.query(sign,.6,-.45)['bank_rad'],.15)

def test_speed_interpolation_support_clamp_and_no_future_state_needed():
 assert hasattr(m,'BankMap'), 'speed-aware inverse missing'
 model=m.BankMap(fixture())
 r=model.query(1,.4,.3)
 assert np.isclose(r['bank_rad'],-.125)
 assert model.query(1,.05,100)['bank_rad']==-.3
 assert model.query(1,.05,100)['curvature_clamped']
 assert model.query(1,.05,100)['speed_clamped']

def test_nonmonotone_calibration_is_rejected_not_sorted_away():
 assert hasattr(m,'BankMap'), 'validation missing'
 import pytest
 x=fixture();x['directions']['forward']['low']['curvature_body_1_m']=[.6,-.2,0]
 with pytest.raises(ValueError):m.BankMap(x)

def test_missing_direction_explicit_and_invalid_input_rejected():
 assert hasattr(m,'BankMap'), 'direction validation missing'
 import pytest
 x=fixture();del x['directions']['reverse'];model=m.BankMap(x)
 with pytest.raises(KeyError):model.query(-1,.2,.1)
 with pytest.raises(ValueError):model.query(1,float('nan'),.1)
