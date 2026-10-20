"""One exact-black endpoint rule, same original texture/height source and CAD producer."""
import copy
import numpy as np
from paired_texture_surface import PairedTextureSurfaceMemory
class PairedCurrentPixelSurfaceMemory(PairedTextureSurfaceMemory):
 def update(self,packet,left_rgb,right_rgb,now,input_hashes,disparity_native):
  t=float(packet['time_s'])
  if not np.isfinite([t,now]).all() or t!=now or t<self._last_time:raise ValueError('causal acquisition before payload')
  images=[np.asarray(v) for v in [left_rgb,right_rgb]]
  if any(v.shape!=(480,640,3) or v.dtype!=np.uint8 for v in images):raise ValueError('native current RGB required')
  for key in ['query_pixel_id','partner_pixel_id']:
   ids=np.asarray(packet[key])
   if ids.ndim!=1 or not np.issubdtype(ids.dtype,np.integer) or np.any((ids<0)|(ids>=480*640)):raise ValueError('current exposure pixel IDs')
  record=super().update(packet,left_rgb,right_rgb,now,input_hashes,disparity_native);q=record['query_pixel_id'];r=record['partner_pixel_id'];keep=~np.all(images[0][q//640,q%640]==0,axis=1)&~np.all(images[1][r//640,r%640]==0,axis=1)
  removed=int((~keep).sum())
  for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:record[k]=record[k][keep].copy()
  record.update(exactblack_endpoints_removed=removed,pixel_support_not_correspondence_certificate=True,source_model_admitted=False,measured_geometry_admitted=False)
  self._records[-1]=copy.deepcopy(record)
  return record
