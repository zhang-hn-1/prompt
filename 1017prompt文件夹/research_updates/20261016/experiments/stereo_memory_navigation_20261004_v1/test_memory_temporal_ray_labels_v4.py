import numpy as np
import pytest
from test_memory_temporal_ray_labels_v2 import inputs

def call(a):
 from memory_temporal_ray_labels_v2 import label_ray
 return label_ray(**a,memory_origin=np.array([-23.3,-20.]),memory_resolution=.1)

def placed(x=0.):
 a=inputs();a['camera_usd_world'][:3,3]=[-23.3+x,-20.,1.]
 a['point_world']=np.array([-23.3+x,-20.,0.])
 return a

def test_real_cell_boundary_rejects_same_floor_bin():
 assert call(placed(.0498))['reason']=='different_world_cell'

def test_real_cell_accepts_across_floor_boundary():
 assert call(placed(-.0004))['label']=='ground_support'

@pytest.mark.parametrize('p',[[.05,.15],[-.05,-.15],[.049,.051]])
def test_cells_match_existing_memory_convention(p):
 from memory_temporal_ray_labels_v2 import memory_cells
 from spatial_memory import SpatialMemory
 origin=np.array([-23.3,-20.]);xy=origin+np.array(p)
 assert np.array_equal(memory_cells(xy,origin,.1),SpatialMemory(origin,2,.1).cells(xy))

@pytest.mark.parametrize('reason',['behind_camera','offscreen','masked','invalid_depth','occluded','disoccluded'])
def test_unavailable_is_not_free_or_hazard(reason):
 a=placed()
 if reason=='behind_camera':a['point_world'][2]=2.
 if reason=='offscreen':a['point_world'][0]+=10.
 if reason=='masked':a['self_mask'][240,320]=True
 if reason=='invalid_depth':a['depth_m'][240,320]=np.nan
 if reason=='occluded':a['depth_m'][240,320]=.5
 if reason=='disoccluded':a['depth_m'][240,320]=2.
 r=call(a);assert r['reason']==reason and r['label']=='unlabelled'
 assert not r['free_volume_certified'] and r['offline_teacher_only']

@pytest.mark.parametrize('z,reason',[(.98,'visible_same_cell'),(1.02,'visible_same_cell'),(np.nextafter(.98,0),'occluded'),(np.nextafter(1.02,2),'disoccluded')])
def test_depth_tolerance_inclusive_boundary(z,reason):
 a=placed();a['camera_usd_world'][2,3]=1.2;a['point_world'][2]=.2;a['depth_m'][240,320]=z
 assert call(a)['reason']==reason

@pytest.mark.parametrize('bad',['reflection','bottom','rotation'])
def test_nonrigid_or_nonhomogeneous_transform_rejected(bad):
 a=placed()
 if bad=='reflection':a['camera_usd_world'][0,0]=-1.
 if bad=='bottom':a['camera_usd_world'][3,0]=.1
 if bad=='rotation':a['camera_usd_world'][0,0]=2.
 with pytest.raises(ValueError):call(a)

@pytest.mark.parametrize('origin,res',[(np.array([0.,np.nan]),.1),(np.zeros(2),0.),(np.zeros(3),.1)])
def test_explicit_memory_geometry_validated(origin,res):
 from memory_temporal_ray_labels_v2 import label_ray
 with pytest.raises(ValueError):label_ray(**inputs(),memory_origin=origin,memory_resolution=res)
