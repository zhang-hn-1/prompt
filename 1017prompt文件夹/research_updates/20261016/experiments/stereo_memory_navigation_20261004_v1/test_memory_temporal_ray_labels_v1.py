import numpy as np
import pytest

def inputs():
 depth=np.full((480,640),np.nan);depth[240,320]=1.
 return dict(point_world=np.array([0.,0.,-1.]),camera_usd_world=np.eye(4),depth_m=depth,self_mask=np.zeros(depth.shape,bool),past_time_s=0.,current_time_s=.5)

def test_hidden_masked_invalid_and_disoccluded_never_labels():
 from memory_temporal_ray_labels_v1 import label_ray
 for reason in ('occluded','masked','invalid_depth','disoccluded'):
  a=inputs()
  if reason=='occluded':a['depth_m'][240,320]=.5
  if reason=='masked':a['self_mask'][240,320]=True
  if reason=='invalid_depth':a['depth_m'][240,320]=np.nan
  if reason=='disoccluded':a['depth_m'][240,320]=2.
  r=label_ray(**a);assert r['label']=='unlabelled' and r['reason']==reason

def test_projection_and_surface_height_diagnostic():
 from memory_temporal_ray_labels_v1 import label_ray
 a=inputs();a['camera_usd_world'][2,3]=1.;a['point_world'][2]=0.
 r=label_ray(**a);assert r['reason']=='visible_same_cell' and r['label']=='ground_support'
 assert not r['free_volume_certified']
 a['point_world'][2]=.2;a['depth_m'][240,320]=.8
 assert label_ray(**a)['label']=='elevated_surface_support'

@pytest.mark.parametrize('key,val',[('current_time_s',0.),('past_time_s',float('nan'))])
def test_noncausal_or_nonfinite_teacher_pair_rejects(key,val):
 from memory_temporal_ray_labels_v1 import label_ray
 a=inputs();a[key]=val
 with pytest.raises(ValueError):label_ray(**a)
