import importlib.util,numpy as np
assert importlib.util.find_spec('visible_target') is not None,'visible target missing'
from visible_target import rasterize
# Four identical ground / obstacle samples make a labelled visible cell; no hit means unknown.
a=rasterize(np.array([[1.,0.,0.]]*4+[[2.,0.,.7]]*4),np.zeros(3),0.)
assert a[10,30]==1 and a[20,30]==3 and (a==0).sum()==2398
assert (rasterize(np.empty((0,3)),np.zeros(3),0.)==0).all()
b=rasterize(np.array([[0.,1.,.2]]*4),np.zeros(3),np.pi/2);assert b[10,30]==2
c=rasterize(np.array([[4.,0.,.2]]*4+[[-.1,0.,.2]]*4),np.zeros(3),0.);assert not c.any()
d=rasterize(np.array([[1.,0.,.2]]*3),np.zeros(3),0.);assert not d.any()
e=rasterize(np.array([[1.,0.,0.]]*4+[[1.,0.,1.2]]*4),np.zeros(3),0.);assert e[10,30]==4
print('6 visibility/geometry cases passed')
