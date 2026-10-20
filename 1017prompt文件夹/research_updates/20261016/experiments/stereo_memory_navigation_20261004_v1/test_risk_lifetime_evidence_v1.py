import json
from pathlib import Path
import numpy as np
import pytest
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence
D=Path(__file__).parent/'sgbm_baseline35_e1_coverage_run_v1'
def test_actual_raw_contact_schema_and_complete_existing_35():
 s=json.loads((D/'states.json').read_text());c=json.loads((D/'physics_contacts.json').read_text())
 n=len(np.load(D/'requests.npz')['requests']);p=np.load(D/'physics.npz')['simulation_time_s']
 result=complete_execution_evidence(s,c,p,n)
 assert len(result)==2101 and result[0]['collision'] is False

def test_missing_available_contact_is_not_complete_evidence():
 s=json.loads((D/'states.json').read_text());c=json.loads((D/'physics_contacts.json').read_text())
 n=len(np.load(D/'requests.npz')['requests']);p=np.load(D/'physics.npz')['simulation_time_s']
 with pytest.raises(ValueError,match='contact'):
  complete_execution_evidence(s,c[:-1],p,n)
