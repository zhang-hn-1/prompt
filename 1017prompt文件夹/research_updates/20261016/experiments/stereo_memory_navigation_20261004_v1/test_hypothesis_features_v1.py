import numpy as np
import pytest
from hypothesis_features_v1 import build_features, scored_hypotheses

def sample():
 w=np.array([[1.,0.,.5],[1.,.1,.6]]);p=np.ones((2,2,2),bool);u=np.zeros_like(p);d=np.ones(p.shape);c=np.array([[[0.,-.3,.4],[0.,.3,.4]],[[.01,-.3,.4],[.01,.3,.4]]]);seed=np.array([[1.,0.,.5],[1.,.1,.6]])
 return [w,p,u,d,c,seed,np.array([.04,.06]),np.array([0.,0.]),1.,np.array([.5,1.])]
def test_schema_finite_and_fixed_features():
 x=build_features(*sample());assert x.shape==(2,14) and np.isfinite(x).all();np.testing.assert_allclose(x[:,:6],[[1,1,0,0,1,0]]*2)
def test_world_xy_translation_does_not_change_features():
 a=sample();x=build_features(*a);shift=np.array([5.,-8.]);a[0][:,:2]+=shift;a[4][:,:,:2]+=shift;a[5][:,:2]+=shift;a[7]+=shift;np.testing.assert_allclose(build_features(*a),x,atol=1e-12)
def test_future_view_rejected_before_payload():
 a=sample();a[-1][1]=1.1;a[0]=object()
 with pytest.raises(ValueError,match='future'):build_features(*a)
def test_empty_seed_not_prior_measurement():
 a=sample();a[5]=np.empty((0,3));a[6]=np.empty(0)
 with pytest.raises(ValueError,match='seed'):build_features(*a)
def test_output_keeps_MODEL_type_and_no_pixel_IDs():
 r=scored_hypotheses(np.array([[1,0,.5],[2,0,.5]]),np.array([.49,.5]),1.)
 assert not r['measured_geometry'] and not r['certified_free'] and r['model_hypothesis']
 assert len(r['raw_world_voxel_centres'])==1 and 'query_pixel_id' not in r
