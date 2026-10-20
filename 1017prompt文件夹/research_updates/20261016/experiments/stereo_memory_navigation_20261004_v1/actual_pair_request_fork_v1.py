"""Observed pair request divergence; never use an unmatched shadow as intervention."""
import numpy as np

def first_actual_request_fork(a,b):
 a=np.asarray(a);b=np.asarray(b)
 if any(x.ndim!=2 or x.shape[1]!=10 or not np.isfinite(x).all() for x in (a,b)):raise ValueError('finite paired actual request arrays N10')
 n=min(len(a),len(b));changes=np.flatnonzero(np.any(a[:n]!=b[:n],axis=1));return None if not len(changes) else int(changes[0])
