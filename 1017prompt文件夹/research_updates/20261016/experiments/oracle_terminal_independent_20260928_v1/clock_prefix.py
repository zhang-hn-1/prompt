import numpy as np

def prefix_matches(a,b,n):
 result={}
 for k in b:
  x=a[k][:n];y=b[k]
  if k=='raw_simulation_time_s':
   result[k]=bool(np.allclose(x-x[0],y-y[0],rtol=0,atol=1e-8))
  elif k=='raw_physics_step_index':result[k]=bool(np.array_equal(x-x[0],y-y[0]))
  else:result[k]=bool(np.array_equal(x,y))
 return result
