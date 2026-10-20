import pytest
from audit_reverse_main_bank_response_v1 import pid_snapshot

def test_preinstallation_snapshot_not_fake_zero():
 assert pid_snapshot(None,0)=={}
 with pytest.raises(ValueError):pid_snapshot(None,1)

def test_missing_runtime_field_not_silent_unavailable():
 with pytest.raises(ValueError):pid_snapshot({'pid':{}},1)
 assert pid_snapshot({'pid':{'last':{'bank_rad':.1,'desired_bank_rad':.2,'error_rad':.1}}},1)['error_rad']==.1
