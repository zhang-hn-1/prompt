import numpy as np
from surface_identity import categories
from category_projection import raster

scene=dict(ground_z_m=0,ground_xy_min_m=[-30,-30],ground_xy_max_m=[30,30],shapes=[dict(type='Sphere',centre=[2,0,.5],rotation=np.eye(3).tolist(),radius=.5),dict(type='Cylinder',centre=[4,0,.5],rotation=np.eye(3).tolist(),radius=.3,height=1)])
p=np.array([[0,0,0],[2.5,0,.5],[2,0,0],[0,0,.4],[4.3,0,.5],[4,0,1],[4.3,0,0]])
a,c=categories(p,scene);assert a.tolist()==[1,2,0,0,2,2,0]
# Category, not estimated height, decides occupancy identity.
assert raster(np.array([[1,0,.4]]*4),np.ones(4,np.uint8),np.zeros(3),0)[10,30]==1
assert raster(np.array([[1,0,0]]*4),np.full(4,2,np.uint8),np.zeros(3),0)[10,30]==2
assert not raster(np.array([[1,0,.7]]*4),np.zeros(4,np.uint8),np.zeros(3),0).any()
assert not raster(np.array([[1,0,.4]]*3),np.full(3,2,np.uint8),np.zeros(3),0).any()
print('7 authored identities + 4 category/height/unknown/count cases pass')
