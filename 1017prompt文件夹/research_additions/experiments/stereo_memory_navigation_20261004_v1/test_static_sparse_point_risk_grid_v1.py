import numpy as np,pytest
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from partial_point_risk_grid import PartialPointRiskGrid

class Poison:
 def __array__(self,*args,**kwargs):raise AssertionError('future payload read')

def record(t,world=None):return dict(time_s=t,world=np.array([[0.,0.,.4]]) if world is None else world)
def test_static_requires_explicit_assumption():
 with pytest.raises(ValueError):StaticSparsePointRiskGrid([-20,-20],10,static_scene_assumption=False)
def test_past_obstacle_persists_without_new_birth_or_free():
 g=StaticSparsePointRiskGrid([-20,-20],10,static_scene_assumption=True);base=PartialPointRiskGrid([-20,-20],10)
 for a in [g,base]:a.consume_records([record(0.)],0.,[1.,1.,.4])
 np.testing.assert_array_equal(g.labels,base.labels)
 for a in [g,base]:a.consume_records([],11.,[1.,1.,.4])
 assert np.any(g.labels==1) and not np.any(base.labels==1)
 assert np.isfinite(g.hazard_acquisition_time).sum()==1 and g.hazard_acquisition_time[200,200]==0.
 assert g.codes[200,200]==9 and not np.any(g.labels==0)
 assert not np.isfinite(g.memory.floor_time).any()
 assert g.diagnostics()['static_scene_assumption'] and not g.diagnostics()['full_3d_certified']
def test_future_rejected_before_payload_and_state_unchanged():
 g=StaticSparsePointRiskGrid([-20,-20],10,static_scene_assumption=True);g.consume_records([record(0.)],0.,[0.,0.,.4]);old=g.labels.copy()
 with pytest.raises(ValueError):g.consume_records([record(2.,Poison())],1.,[0.,0.,.4])
 np.testing.assert_array_equal(g.labels,old)
def test_new_current_point_updates_only_its_birth_and_new_instance_resets():
 g=StaticSparsePointRiskGrid([-20,-20],10,static_scene_assumption=True);g.consume_records([record(0.)],0.,[0.,0.,.4]);g.consume_records([record(12.)],12.,[0.,0.,.4]);assert g.hazard_acquisition_time[200,200]==12. and g.codes[200,200]==8
 fresh=StaticSparsePointRiskGrid([-20,-20],10,static_scene_assumption=True);assert not np.isfinite(fresh.hazard_acquisition_time).any()
