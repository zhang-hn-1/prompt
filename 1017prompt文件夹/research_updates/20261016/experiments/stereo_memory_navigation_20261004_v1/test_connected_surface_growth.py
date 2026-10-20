import numpy as np
from connected_surface_growth import select_connected
# Missing transitive growth must fail this independent literal chain fixture.
def test_chain_extends_observed_structure_but_not_disconnected_component():
 seed=np.array([[0.,0.,1.]]);q=np.array([[.09,0,1.],[.18,0,1.],[.7,0,1.],[.79,0,1.]])
 assert select_connected(seed,q).tolist()==[True,True,False,False]
# Diagonal distance/coordinate mistake must not create a connection.
def test_three_dimensional_gap_cannot_be_bridged_by_just_XY_overlap():
 assert select_connected(np.array([[0.,0,1.]]),np.array([[0,0,1.3],[.01,0,1.3]])).tolist()==[False,False]
# Queue order must not change closure or fabricate nodes through NaNs.
def test_order_invariance_and_invalid_nodes():
 seed=np.array([[0.,0.,1.]]);q=np.array([[.18,0,1.],[np.nan,0,1.],[.09,0,1.]])
 assert select_connected(seed,q).tolist()==[True,False,True]
 assert select_connected(seed,q[::-1]).tolist()==[True,False,True]
 assert select_connected(np.empty((0,3)),q).tolist()==[False,False,False]
