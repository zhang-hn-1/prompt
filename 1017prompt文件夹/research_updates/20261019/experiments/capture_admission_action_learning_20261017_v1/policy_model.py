"""Only numerical causal features -> imitation action score, not calibrated risk."""
import numpy as np
from scipy.special import expit
class AdmissionActionModel:
    def __init__(self,weights):
        self.mean=np.asarray(weights['mean'],float);self.scale=np.asarray(weights['scale'],float)
        self.coef=np.asarray(weights['coef'],float);self.intercept=float(weights['intercept'])
        if any(x.shape!=(24,) or not np.isfinite(x).all() for x in [self.mean,self.scale,self.coef]) or not (self.scale>0).all() or not np.isfinite(self.intercept):raise ValueError('frozen finite24 action model required')
    def score(self,features):
        if not isinstance(features,np.ndarray) or features.ndim!=2 or features.shape[1]!=24:raise ValueError('only Nx24 numerical feature input')
        valid=np.isfinite(features).all(1);out=np.full(len(features),np.nan)
        out[valid]=expit(((features[valid]-self.mean)/self.scale)@self.coef+self.intercept)
        return out
