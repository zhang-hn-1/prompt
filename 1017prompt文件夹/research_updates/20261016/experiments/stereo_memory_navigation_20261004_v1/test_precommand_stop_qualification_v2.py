import numpy as np
import pytest
from precommand_stop_qualification_v2 import qualify

def test_fifth_capture_precedes_new_command_and_belongs_to_stop_measurement():
 assert qualify(np.array([.2,.04,.03,.02,.01,.02,.9]),0,5)['qualified_frame']==5

def test_rebound_resets_actual_world_speed_counter():
 assert qualify(np.array([.04,.04,.04,.1,.04,.04,.04,.04,.04]),0,8)['qualified_frame']==8

def test_four_low_samples_are_insufficient_even_at_record_end():
 assert qualify(np.array([.2,.04,.03,.02,.01]),0,4)['qualified_frame'] is None

def test_boundary_includes_measurement_but_never_later_physics():
 a=np.array([.2,.04,.03,.02,.01,.02,.9,.9]);b=a.copy();b[6:]=np.nan
 assert qualify(a,0,5)==qualify(b,0,5)==qualify(a[:6],0,5)

def test_invalid_current_measurement_is_not_low_speed():
 with pytest.raises(ValueError):qualify(np.array([.2,np.nan,.02]),0,2)
