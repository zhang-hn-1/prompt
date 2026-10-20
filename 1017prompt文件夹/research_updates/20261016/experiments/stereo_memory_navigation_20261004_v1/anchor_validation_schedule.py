"""One known bounded sensing command sequence, not a navigation policy."""
import numpy as np

def request(time_s,kind='near'):
 t=float(time_s)
 if not np.isfinite(t) or t<0 or kind!='near':raise ValueError('finite current clock, near sequence only')
 main=-.3 if 6-1e-6<=t<12-1e-6 else .3 if 18-1e-6<=t<24-1e-6 else 0.
 aux=np.deg2rad(10.) if 18-1e-6<=t<24-1e-6 else 0.
 return np.tile(np.array([main,aux],np.float32),5)
