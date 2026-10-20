"""Real closed local feedback rows; label audit, not another physical run."""
from pathlib import Path
import json,numpy as np
from measured_speed_consumer_v1 import FEEDFORWARD,KP,TARGET
H=Path(__file__).parent

def test_promoted_current_proprio_norm_reconstructs_actual_float32_requests():
 d=H/'measured_speed_fixed_reference_paced_run_v1';states=json.loads((d/'states.json').read_text());rows=json.loads((d/'policy_records.json').read_text())
 from assess_measured_speed_navigation_e0_v1 import reconstruct_current_speed
 for f in (91,93,99,105):
  row=rows[f];speed=reconstruct_current_speed(states[f]['velocity']);expected=np.float32(np.clip(FEEDFORWARD+KP*(TARGET-speed),.5,1.))
  assert abs(speed-row['measured_speed_worldXY_m_s'])<1e-12
  assert np.all(np.asarray(row['request'],np.float32)[::2]==expected)

def test_point_identity_ignores_only_absolute_provenance_location():
 from assess_measured_speed_navigation_e0_v1 import meaningful_snapshot_equal
 a=dict(current_count=1,records=[dict(frame=5,time_s=.5,world=[[1.,2.,3.]],query_pixel_id=[2],partner_pixel_id=[3],layer='TENTATIVE',input_sha256={'/armA/source.npz':'a'})])
 b=json.loads(json.dumps(a));b['records'][0]['input_sha256']={'/armB/source.npz':'a'}
 assert meaningful_snapshot_equal(a,b)
 b['records'][0]['world'][0][0]=1.01
 assert not meaningful_snapshot_equal(a,b)
