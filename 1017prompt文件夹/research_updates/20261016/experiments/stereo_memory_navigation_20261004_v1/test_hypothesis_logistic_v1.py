import numpy as np
import pytest
from hypothesis_logistic_v1 import fit_historical,predict

def test_fresh_whole_sequence_cannot_fit():
 with pytest.raises(ValueError,match='historical'):fit_historical({'hypothesis_learning_H1_run_v1':(np.zeros((2,14)),np.array([0,1]))})
def test_both_classes_required_no_fallback_model():
 with pytest.raises(ValueError,match='both classes'):fit_historical({'flat_ground_pose_D3_run_v1':(np.zeros((2,14)),np.array([1,1]))})
def test_simple_distinguishable_response_finite_and_ordered():
 x=np.zeros((20,14));x[:,0]=np.linspace(-2,2,20);y=(x[:,0]>0).astype(int);m=fit_historical({'flat_ground_pose_D3_run_v1':(x,y)});p=predict(m,x)
 assert np.isfinite(p).all() and p[0]<.5<p[-1]
 before=m['mean'].copy();predict(m,x+100);np.testing.assert_array_equal(m['mean'],before)
