import numpy as np,cv2
from strict_bilinear_support import unmasked_linear_support

def test_smallest_nonzero_masked_tap_is_rejected():
 m=np.zeros((2,2),bool);m[1,1]=True;x=np.array([[1/32]],np.float32);y=x.copy()
 assert not unmasked_linear_support(m,x,y)[0,0]

def test_zero_weight_masked_neighbor_does_not_invalidate_sample():
 m=np.zeros((2,2),bool);m[1,1]=True;x=np.array([[0]],np.float32);y=x.copy()
 assert unmasked_linear_support(m,x,y)[0,0]

def test_outside_and_finite_valid_sample():
 m=np.zeros((2,2),bool);x=np.array([[-.1,0]],np.float32);y=np.array([[0,0]],np.float32)
 assert unmasked_linear_support(m,x,y).tolist()==[[False,True]]
