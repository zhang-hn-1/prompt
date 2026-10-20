import numpy as np
import pytest
from provided_historical_model import installable_model

def seed():
 return dict(time_s=14.,hypothesis_birth_time_s=14.,source_model_birth_time_s=8.,source_seed_time_s=8.,model_hypothesis=True,measured_geometry=False,certified_free=False,static_persistence_prior=True,possible_xy=np.array([[0.,1.]]),raw_world_voxel_centres=np.array([[0.,1.,.2]]))

def test_instantiation_is_model_not_new_exposure_and_geometry_immutable():
 s=seed();r=installable_model(s,14.,prior_sequence='old',prior_sha256='a'*64)
 for k in ['possible_xy','raw_world_voxel_centres']:np.testing.assert_array_equal(s[k],r[k]);assert not np.shares_memory(s[k],r[k])
 assert r['source_seed_time_s']==r['source_model_birth_time_s']==-1.;assert r['time_s']==r['hypothesis_birth_time_s']==14.;assert r['source_consumer']=='PROVIDED_HISTORICAL_MODEL_CONTROL_DIAGNOSTIC';assert r['provided_model_provenance']['prior_source_model_birth_time_s']==8.;assert not r['provided_model_provenance']['current_observation'];assert not r['certified_free']

def test_bad_measurement_or_identity_rejected_before_adaptation():
 s=seed();s['measured_geometry']=True
 with pytest.raises(ValueError):installable_model(s,14.,prior_sequence='old',prior_sha256='a'*64)
 with pytest.raises(ValueError):installable_model(seed(),14.,prior_sequence='',prior_sha256='a'*64)
 with pytest.raises(ValueError):installable_model(seed(),float('nan'),prior_sequence='old',prior_sha256='a'*64)
