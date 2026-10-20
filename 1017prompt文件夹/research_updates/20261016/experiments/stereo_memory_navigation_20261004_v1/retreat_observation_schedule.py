"""Fixed one-sequence sensing probe; no navigation or future-feedback input."""
import numpy as np

def request(current_time_s):
 t=float(current_time_s)
 if not np.isfinite(t) or t<0:raise ValueError('finite nonnegative current clock required')
 # Simulation uses float32-derived .1s clocks; align exactly to frozen ticks.
 main=-.3 if 6-1e-6<=t<12-1e-6 else 0.
 return np.tile(np.array([main,0.],dtype=np.float32),5)
