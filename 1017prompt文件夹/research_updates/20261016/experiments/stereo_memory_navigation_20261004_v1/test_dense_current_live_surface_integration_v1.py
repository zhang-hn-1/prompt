"""Real existing texture/height memory consumer, without CAD or simulation."""
import io
import numpy as np
from test_dense_current_source_packet_v1 import fixture
from test_dense_current_live_transport_v1 import CAD

def test_dense_packet_survives_serialization_and_existing_surface_consumer():
 from dense_current_live_transport_v1 import paired_sources
 from paired_texture_surface import PairedTextureSurfaceMemory
 prediction,pose,_,_=fixture()
 pose['left_camera_usd_world'][2,3]=1.5
 pose['right_camera_usd_world'][2,3]=1.5
 shadow,current,_=paired_sources(prediction,pose,np.zeros((5,17)),CAD(),.5)
 stream=io.BytesIO();np.savez_compressed(stream,**current);stream.seek(0)
 restored=dict(np.load(stream))
 rng=np.random.default_rng(19);rgb=rng.integers(0,256,(480,640,3),dtype=np.uint8)
 mem=PairedTextureSurfaceMemory()
 record=mem.update(restored,rgb,rgb,.5,{'current_sensor_packet':'test_identity'},prediction['disparity_native_px'])
 assert len(record['world'])==2
 assert record['query_pixel_id'].tolist()==current['query_pixel_id'].tolist()
 assert record['partner_pixel_id'].tolist()==current['partner_pixel_id'].tolist()
 assert record['input_hashes']=={'current_sensor_packet':'test_identity'}
 assert record['flat_floor_height_is_prior'] and record['provisional_occupied_surface']
 assert not record['certified_free'] and not record['unseen_volume_observed']
 # Consumer keeps a copy; later transport mutations cannot alter prior evidence.
 restored['world'][:]=999
 assert np.array_equal(mem.at(.5)[0]['world'],current['world'])
 assert shadow['query_pixel_id'].tolist()==[400*640+320]
