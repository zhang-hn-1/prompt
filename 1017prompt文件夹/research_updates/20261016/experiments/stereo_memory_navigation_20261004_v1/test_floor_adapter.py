import numpy as np
import pytest
from floor_adapter import fold_affine,apply_head,features_at_pixels

def test_scalar_standardization_fold_matches_hand_logit():
    h=fold_affine([.2,.3],[2.,4.],[1.,-2.],.5);p=apply_head(dict(features=np.array([[4.2,4.3]])),h);assert p[0]==pytest.approx(1/(1+np.exp(-.5)))
def test_probabilities_rgb_features_are_pixel_only():
    probs=np.zeros((150,2,3));probs[3]=.9;rgb=np.zeros((2,3,3),np.uint8);rgb[1,2]=[255,0,128];x=features_at_pixels(probs,rgb,np.array([1]),np.array([2]));assert x.shape==(1,153);assert x[0,3]==pytest.approx(.9);assert np.allclose(x[0,-3:],[1,0,128/255])
def test_inference_packet_rejects_label_or_pose():
    h=fold_affine([0],[1],[1],0)
    with pytest.raises(ValueError):apply_head(dict(features=np.ones((2,1)),labels=np.array([1,1])),h)
def test_features_and_head_not_mutated():
    h=fold_affine([0],[1],[1],0);x=np.array([[2.]]);before=x.copy();w=h['weight'].copy();p=apply_head(dict(features=x),h);p[:]=0;assert np.array_equal(x,before);assert np.array_equal(h['weight'],w)
def test_nonfinite_feature_rejected():
    h=fold_affine([0],[1],[1],0)
    with pytest.raises(ValueError):apply_head(dict(features=np.array([[np.nan]])),h)
def test_bad_feature_dimension_rejected():
    h=fold_affine([0],[1],[1],0)
    with pytest.raises(ValueError):apply_head(dict(features=np.zeros((2,2))),h)

def test_balanced_logistic_objective_literal_zero_parameters():
    from floor_adapter import balanced_logistic_objective
    loss,g=balanced_logistic_objective(np.zeros(2),np.array([[1.],[2.]]),np.array([0.,1.]),np.ones(2))
    assert loss==pytest.approx(2*np.log(2));assert np.allclose(g,[-.5,0.])
def test_logistic_gradient_finite_difference_nonzero():
    from floor_adapter import balanced_logistic_objective
    x=np.array([[1.,.4],[-.2,2.],[.3,-.1]]);y=np.array([0.,1.,1.]);w=np.array([1.5,.75,.75]);theta=np.array([.2,-.3,.4]);loss,g=balanced_logistic_objective(theta,x,y,w)
    for j in range(3):
        delta=np.eye(3)[j]*1e-6;a=balanced_logistic_objective(theta+delta,x,y,w)[0];b=balanced_logistic_objective(theta-delta,x,y,w)[0];assert g[j]==pytest.approx((a-b)/2e-6,abs=1e-6)

def test_dense_fold_matches_same_pixel_matrix():
    from frozen_floor_features import dense_head_probability
    rng=np.random.default_rng(1);probs=rng.random((150,2,3)).astype(np.float32);probs/=probs.sum(axis=0);rgb=rng.integers(0,256,(2,3,3),dtype=np.uint8);h=fold_affine(np.zeros(153),np.ones(153),rng.normal(size=153),.2)
    yy,xx=np.mgrid[:2,:3];x=features_at_pixels(probs,rgb,yy.ravel(),xx.ravel());assert np.allclose(dense_head_probability(probs,rgb,h),apply_head(dict(features=x),h).reshape(2,3),atol=1e-7)
