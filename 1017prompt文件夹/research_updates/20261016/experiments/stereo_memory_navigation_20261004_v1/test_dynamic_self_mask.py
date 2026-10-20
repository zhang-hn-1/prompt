import numpy as np
import pytest
from dynamic_self_mask import joint_transform,RobotSelfMask,joints_from_proprio

def test_joint_transform_rotation_and_local_frames():
 T=joint_transform('X',np.pi/2,[0,0,0],[1,0,0,0],[0,0,0],[1,0,0,0]);np.testing.assert_allclose((T@np.array([0,1,0,1]))[:3],[0,0,1],atol=1e-12)
 with pytest.raises(ValueError):joint_transform('X',np.nan,[0,0,0],[1,0,0,0],[0,0,0],[1,0,0,0])

def test_current_proprio_only_not_future_or_truth():
 x=np.zeros((5,17));x[-1,9:12]=[.1,.2,.3];np.testing.assert_equal(joints_from_proprio(x),[.1,.2,.3]);x[-1,9]=np.nan
 with pytest.raises(ValueError):joints_from_proprio(x)
 with pytest.raises(ValueError):joints_from_proprio(np.zeros((5,12)))

def test_base_only_matches_frozen_mask_exactly():
 from pathlib import Path
 H=Path(__file__).resolve().parent;old=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');cad=RobotSelfMask()
 m=cad.project(np.zeros(3),parts=('base_link',),include_frozen=False)
 for eye in ('left','right'):np.testing.assert_equal(m[eye],old[eye])

def test_joint_conditioned_candidate_keeps_frozen_base():
 cad=RobotSelfMask();a=cad.project(np.zeros(3));b=cad.project(np.array([.8,-.2,1.2]))
 for eye in ('left','right'):
  assert a[eye].dtype==bool and a[eye].shape==(480,640);assert np.all(a[eye][cad.frozen[eye]]);assert np.all(b[eye][cad.frozen[eye]])
 assert any(not np.array_equal(a[eye],b[eye]) for eye in ('left','right'))
