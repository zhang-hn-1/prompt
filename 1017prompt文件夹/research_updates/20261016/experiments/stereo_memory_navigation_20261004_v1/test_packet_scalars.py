import numpy as np
from packet_scalars import normalize_npz_packet
from dense_source_points import records
from point_support_shadow import PointSupportMap

def test_npz_wire_scalar_ids_are_hashable_runtime_types(tmp_path):
 p=dict(world=np.array([[0.,0.,.5]]),sigma_m=np.array([.01]),query_pixel_id=np.array([18]),partner_pixel_id=np.array([10]),frame=0,time_s=0.,pose_sigma_m=0.,source_model_admitted=False,certified_free=False,frame_width=8);f=tmp_path/'p.npz';np.savez(f,**p);d=np.load(f);wire={k:d[k] for k in d.files};n=normalize_npz_packet(wire);assert isinstance(n['frame'],int) and isinstance(n['time_s'],float) and isinstance(n['certified_free'],bool);assert n['world'] is wire['world'];assert isinstance(wire['frame'],np.ndarray);m=PointSupportMap([-2,-2],(40,40),.1);m.update(records(n),0,0);assert len(m.records)==1 and m.snapshot(0)['labels'][20,20]==1
