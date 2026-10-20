import numpy as np
def schedule(arm):
 if arm not in ('low','high'):raise ValueError('one frozen low/high pair only')
 a=np.zeros((300,10),np.float32);a[60:220,::2]=.5;a[60:220,1::2]=-np.pi/6
 if arm=='high':a[120:220,::2]=1.
 return a
