import numpy as np
from npz_packet_scalars import restore_packet_scalars
from dense_source_points import records
from point_support_shadow import PointSupportMap

def test_npz_scalar_identity_roundtrip_becomes_hashable_without_moving_points():
 p=dict(frame=np.array(5),time_s=np.array(.5),world=np.array([[.1,0,.2]]),sigma_m=np.array([.02]),query_pixel_id=np.array([7]),partner_pixel_id=np.array([8]));q=restore_packet_scalars(p);assert type(q['frame']) is int and type(q['time_s']) is float;assert q['world'] is p['world'];assert isinstance(p['frame'],np.ndarray);m=PointSupportMap([0,0],(3,3),.1);m.update(records(q),.5,0);assert len(m.records)==1
