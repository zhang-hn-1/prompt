"""Offline boolean cell classification, not source-depth or safety admission."""
import numpy as np

def confusion(predicted,truth):
 p=np.asarray(predicted,bool);y=np.asarray(truth,bool)
 if p.shape!=y.shape or p.ndim!=1:raise ValueError('aligned 1D decisions required')
 tp=int((p&y).sum());fp=int((p&~y).sum());fn=int((~p&y).sum());tn=int((~p&~y).sum())
 return dict(TP=tp,FP=fp,FN=fn,TN=tn,precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None,eligible_rows=len(y))
