import numpy as np
import pytest
from bilateral_hypothesis_consistency import bilateral_hypothesis_pairs

def case(depth=2.):
    # Use native K, choose rays separated by one2px lattice step through baseline.
    focal = 584.8888888888889
    left_pose = np.diag([1., -1., -1., 1.])
    right_pose = left_pose.copy()
    right_pose[0, 3] = 2.*2./focal
    left = dict(current_time_s=1., current_frame=10, current_uv=np.array([[320,240]]),
                world=np.array([[(320-319.5)/focal*depth,(240-239.5)/focal*depth,depth]]))
    right = dict(current_time_s=1., current_frame=10, current_uv=np.array([[318,240]]),
                 world=np.array([[right_pose[0,3]+(318-319.5)/focal*depth,(240-239.5)/focal*depth,depth]]))
    return left,right,left_pose,right_pose

def test_true_plane_reciprocal_same_epoch_pair():
    pairs,gap=bilateral_hypothesis_pairs(*case(),1.)
    np.testing.assert_array_equal(pairs,[[0,0]])
    np.testing.assert_allclose(gap,[0.],atol=1e-12)

def test_shared_twenty_cm_bias_is_not_certified_by_agreement():
    pairs,gap=bilateral_hypothesis_pairs(*case(2.2),1.)
    np.testing.assert_array_equal(pairs,[[0,0]])
    assert gap[0]<.1 # Both20cmwrong butsame-sourceagreement retains them.

def test_missing_retained_partner_is_not_filled_from_dense_depth():
    left,right,lp,rp=case()
    right['current_uv']=np.empty((0,2),int);right['world']=np.empty((0,3))
    pairs,gap=bilateral_hypothesis_pairs(left,right,lp,rp,1.)
    assert pairs.shape==(0,2) and gap.shape==(0,)

def test_future_clock_rejected_before_world_payload():
    class PayloadTrap(dict):
        def __getitem__(self,key):
            if key in ('world','current_uv'):raise AssertionError('future payload was read')
            return super().__getitem__(key)
    left,right,lp,rp=case();right=PayloadTrap(right);right['current_time_s']=1.1
    with pytest.raises(ValueError,match='exposure'):
        bilateral_hypothesis_pairs(left,right,lp,rp,1.)
