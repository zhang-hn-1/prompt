from pathlib import Path
import sys,numpy as np,pytest
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
sys.path.insert(0,str(H.parent/'clean_visual_hard0_intervention_20261003_v1'))
sys.path.insert(0,str(H.parent/'visible_surface_connectivity_20261004_v1'))

def test_pooled_hypothesis_time_does_not_rewrite_original_class_birth():
 from persistent_core_grid import PersistentCoreGrid
 g=PersistentCoreGrid(np.array([-20.,-20.]),60)
 q=dict(time_s=14.,hypothesis_birth_time_s=14.,source_seed_time_s=2.,source_model_birth_time_s=2.,model_hypothesis=True,measured_geometry=False,certified_free=False,static_persistence_prior=True,possible_xy=np.array([[0.,0.]]))
 g.seed(q,14.)
 assert g.birth['source_seed_time_s']==2.
 g.set_source(dict(time_s=14.))
 g.update_points(np.empty((0,3)),np.array([-2.2,0.,.4]),14.)
 assert (g.codes==6).sum()==1 and not np.isfinite(g.memory.hazard_time).any() and not np.isfinite(g.memory.floor_time).any()
 with pytest.raises(ValueError):g.seed(q,14.)
 q['possible_xy'][:]=999
 assert g.birth['possible_xy'][0,0]==0.

def test_nonfinite_consumer_clock_cannot_seed_or_mutate_map():
 from persistent_core_grid import PersistentCoreGrid
 g=PersistentCoreGrid(np.array([-20.,-20.]),60)
 with pytest.raises(ValueError):g.seed(dict(time_s=14.),float('nan'))
 assert g.birth is None
