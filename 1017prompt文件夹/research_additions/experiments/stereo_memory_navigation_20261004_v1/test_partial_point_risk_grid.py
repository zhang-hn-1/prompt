import numpy as np,pytest
from partial_point_risk_grid import PartialPointRiskGrid

def test_future_rejected_before_poison_payload():
 g=PartialPointRiskGrid(np.array([-3.,-3.]),10)
 with pytest.raises(ValueError):g.consume_records([{'time_s':2.}],1.,np.array([-2.2,.15]))
 assert g.birth is None

def test_unknown_no_free_and_birth_not_refreshed():
 g=PartialPointRiskGrid(np.array([-3.,-3.]),10);p=np.array([[0.,0.,.3]]);r={'time_s':1.,'world':p};g.consume_records([r],1.,np.array([-2.2,.15]));assert (g.codes==8).sum()==1;assert set(np.unique(g.labels))=={-1,1};p[0]=100
 g.consume_records([{'time_s':1.,'world':np.array([[0.,0.,.3]])}],1.5,np.array([-2.,.15]));assert (g.codes==9).sum()==1;assert g.hazard_acquisition_time[g.codes==9][0]==1.
 g.consume_records([{'time_s':1.,'world':np.array([[0.,0.,.3]])}],11.0001,np.array([-2.,.15]));assert (g.labels==1).sum()==0;assert g.safe_start_prior.any();assert not (g.labels==0).any()

def test_no_model_volume_expansion_and_invalid_height_rejected():
 g=PartialPointRiskGrid(np.array([-3.,-3.]),10)
 with pytest.raises(ValueError):g.consume_records([{'time_s':0.,'world':np.array([[0.,0.,np.nan]])}],0.,np.array([-2.2,.15]))
 g.consume_records([{'time_s':0.,'world':np.array([[0.,0.,.3]])}],0.,np.array([-2.2,.15]));assert (g.labels==1).sum()==1;assert not (g.codes==6).any();assert not (g.codes==7).any()

def test_empty_json_surface_stays_unknown():
 g=PartialPointRiskGrid(np.array([-3.,-3.]),10);g.consume_records([{'time_s':0.,'world':[]}],0.,np.array([-2.2,.15]));assert (g.labels==-1).all()

def test_compatibility_update_has_same_cells_and_future_envelope_rejected():
 g=PartialPointRiskGrid(np.array([-3.,-3.]),10);root=np.array([-2.2,.15,.4]);r=[{'time_s':1.,'world':[[0.,0.,.3]]}]
 g.set_source({'time_s':1.,'records':r});g.update_points(np.empty((0,3)),root,1.)
 assert (g.codes==8).sum()==1 and g.diagnostics()['source_volume_admitted'] is False
 labels=g.labels.copy();g.set_source({'time_s':2.,'records':None})
 with pytest.raises(ValueError):g.update_points(np.empty((0,3)),root,1.)
 np.testing.assert_array_equal(g.labels,labels)

def test_record_newer_than_envelope_rejected_before_world():
 g=PartialPointRiskGrid(np.array([-3.,-3.]),10);g.set_source({'time_s':0.,'records':[{'time_s':1.}]})
 with pytest.raises(ValueError,match='source envelope'):g.update_points(np.empty((0,3)),np.array([-2.2,.15]),1.)
 assert (g.labels==-1).all()
