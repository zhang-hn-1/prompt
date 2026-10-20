"""Independent bounded spatial-cost candidate. Initial TDD scaffold."""
import numpy as np

def aggregate_cost(volume, p1=.05, p2=.20):
    return np.array(volume, copy=True)

def decode_evidence(volume,bits,raynorm,depths,regularized,*,max_cost=.2,slack=.05,max_interval=.1):
    best=np.argmin(regularized,axis=0)
    return dict(winner_index=best,confidence=np.ones(best.shape,bool))
