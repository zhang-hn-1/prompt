import numpy as np
import bank_mapping as m
from test_bank_mapping import fixture
from test_benchmark_policies import aux30_protocol,valid_observation,straight_oracle_context
from benchmark.policies import OraclePathPolicy

def test_real_oracle_main_and_single_call_kept_while_mapping_changes_steering():
 assert hasattr(m,'make_policy'), 'mapping integration missing'
 cls=m.make_policy(OraclePathPolicy,m.BankMap(fixture()));p=cls(aux30_protocol());b=OraclePathPolicy(aux30_protocol())
 for x in [p,b]:x.reset(straight_oracle_context('forward'))
 obs=valid_observation(position_world_xy=(0,.1),forward_speed_m_s=.2,yaw_rate_rad_s=.1)
 candidate=p.act(obs,(2,-.1)).reshape(-1,2);base=b.act(obs,(2,-.1)).reshape(-1,2)
 np.testing.assert_array_equal(candidate[:,0],base[:,0])
 assert not np.allclose(candidate[:,1],base[:,1])
 assert len(p.tracking_records)==1 and len(p.mapping_records)==1
 row=p.mapping_records[0];assert row['active']
 assert np.isclose(row['new_aux_rad'],row['encoded_aux_rad']+row['outer_yaw_correction_rad'])
 # A goal-return must not reuse previous valid tracking geometry.
 out=p.act(obs,(.1,0));assert np.all(out==0)
 assert not p.mapping_records[-1]['active']

def test_reverse_real_policy_and_unsupported_direction_fallback():
 full=m.BankMap(fixture());payload=fixture();del payload['directions']['reverse'];missing=m.BankMap(payload)
 cls=m.make_policy(OraclePathPolicy,full);fallback=m.make_policy(OraclePathPolicy,missing)
 candidate=cls(aux30_protocol());f=fallback(aux30_protocol());b=OraclePathPolicy(aux30_protocol())
 for p in [candidate,f,b]:p.reset(straight_oracle_context('reverse'))
 obs=valid_observation(position_world_xy=(0,.1),forward_speed_m_s=-.2,yaw_rate_rad_s=.1)
 c=candidate.act(obs,(-2,-.1)).reshape(-1,2);old=b.act(obs,(-2,-.1));same=f.act(obs,(-2,-.1))
 np.testing.assert_array_equal(c[:,0],old.reshape(-1,2)[:,0]);np.testing.assert_array_equal(same,old)
 assert candidate.mapping_records[-1]['active']
 assert candidate.mapping_records[-1]['direction']=='reverse'
 assert f.mapping_records[-1]['fallback']=='unsupported_drive_direction'
