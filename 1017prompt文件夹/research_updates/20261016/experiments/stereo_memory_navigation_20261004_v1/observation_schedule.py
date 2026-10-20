"""Frozen observation motions, only certified safe-start/open-plane diagnostics."""
import numpy as np

def schedule(name):
    a=np.zeros((120,5,2),np.float32)
    if name=='bank':
        a[60:80,:,1]=np.deg2rad(15);a[80:100,:,1]=-np.deg2rad(15)
    elif name in ('forward','reverse'):
        a[60:90,:,0]=.3 if name=='forward' else -.3
    elif name!='zero':raise ValueError(name)
    return a.reshape(120,10)

def start_authorized(certified_free_radius_m,required_radius_m):
    return bool(np.isfinite(certified_free_radius_m) and certified_free_radius_m>=required_radius_m)
