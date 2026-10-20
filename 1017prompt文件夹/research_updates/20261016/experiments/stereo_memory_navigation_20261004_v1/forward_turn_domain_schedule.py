"""One fixed72s coverage sequence, no search or parameter fitting."""
import numpy as np

def schedule():
 a=np.zeros((720,5,2),np.float32);a[60:660,:,0]=.5;a[260:460,:,1]=np.pi/6;a[460:660,:,1]=-np.pi/6;return a.reshape(720,10)
def windows():return ((21.,25.5,0.),(41.,45.5,np.pi/6),(61.,65.5,-np.pi/6))
