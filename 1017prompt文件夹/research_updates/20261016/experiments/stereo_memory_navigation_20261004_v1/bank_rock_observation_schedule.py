import numpy as np

def request(frame):
 if not isinstance(frame,int) or not 0<=frame<140:raise ValueError('only frozen140requests')
 bank=10. if 60<=frame<80 else -10. if 80<=frame<100 else 0.
 return np.tile(np.array([0.,np.deg2rad(bank/.75)],np.float32),5)
