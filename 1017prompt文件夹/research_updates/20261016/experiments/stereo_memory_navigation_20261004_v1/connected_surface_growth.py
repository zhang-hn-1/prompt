"""Initial direct-anchor behavior, before the requested connected extension."""
import numpy as np
from scipy.spatial import cKDTree
def select_connected(seed,points):
 p=np.asarray(points,float);out=np.zeros(len(p),bool);finite=np.isfinite(p).all(1)
 s=np.asarray(seed,float)
 if not len(s) or not finite.any():return out
 s=s[np.isfinite(s).all(1)]
 if not len(s):return out
 nodes=p[finite];tree=cKDTree(nodes);seen=cKDTree(s).query(nodes)[0]<=.1
 pending=list(np.flatnonzero(seen))
 while pending:
  i=pending.pop()
  for j in tree.query_ball_point(nodes[i],.1):
   if not seen[j]:seen[j]=True;pending.append(j)
 out[finite]=seen
 return out
