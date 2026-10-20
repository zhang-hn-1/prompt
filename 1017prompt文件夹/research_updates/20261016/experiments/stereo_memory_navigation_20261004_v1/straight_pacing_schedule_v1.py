import numpy as np
def schedule():
 a=np.zeros((300,10),np.float32);a[60:120,::2]=.5;a[120:220,::2]=1.;return a
