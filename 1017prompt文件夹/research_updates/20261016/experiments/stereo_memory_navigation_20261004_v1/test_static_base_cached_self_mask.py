from pathlib import Path
import numpy as np,pytest
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from static_base_cached_self_mask import StaticBaseCachedSelfMask
H=Path(__file__).resolve().parent
@pytest.fixture(scope='module')
def masks():return RobotSelfMask(),StaticBaseCachedSelfMask()
def test_measured_dynamic_masks_match_original_at_two_distinct_states(masks):
 old,new=masks;prev=None;changed=False
 for i in (150,295):
  q=joints_from_proprio(np.load(H/'live_paired_texture_navigation_run_v1/history'/f'frame_{i:03d}.npz')['proprio_history']);a=old.project(q);b=new.project(q)
  for eye in ('left','right'):np.testing.assert_array_equal(a[eye],b[eye])
  if prev is not None:changed=any(not np.array_equal(a[k],prev[k]) for k in a)
  prev=a
 assert changed,'must exercise genuinely changed dynamic mask, not static outputs'
def test_invalid_current_joints_rejected_and_return_mutation_does_not_change_cache(masks):
 _,new=masks
 for q in ([0,np.nan,0],[0,0]):
  with pytest.raises(ValueError):new.project(q)
 a=new.project([0,0,0]);expected={k:v.copy() for k,v in a.items()}
 for v in a.values():v[:]=False
 b=new.project([0,0,0])
 for k in b:np.testing.assert_array_equal(b[k],expected[k])
def test_nondefault_parts_and_no_frozen_semantics_match_original(masks):
 old,new=masks;q=[.1,-.2,.3]
 for parts in [('base_link',),('link3',),('base_link','link1','link2','link3')]:
  a=old.project(q,parts=parts,include_frozen=False);b=new.project(q,parts=parts,include_frozen=False)
  for k in a:np.testing.assert_array_equal(a[k],b[k])
