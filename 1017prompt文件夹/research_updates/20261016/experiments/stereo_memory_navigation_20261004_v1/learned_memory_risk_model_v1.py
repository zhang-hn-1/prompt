"""One frozen linear risk-insertion score; not calibrated physical safety probability."""
import numpy as np
from learned_memory_risk_features_v1 import FEATURE_NAMES

def predict_features(model,features):
 x=np.asarray(features,float)
 if x.ndim!=2 or x.shape[1]!=len(FEATURE_NAMES) or not np.isfinite(x).all():raise ValueError('current finite feature matrix')
 if model['feature_names']!=FEATURE_NAMES:raise ValueError('frozen feature identity')
 z=(x-np.asarray(model['mean']))/np.asarray(model['scale']);linear=z@np.asarray(model['coef'])+float(model['intercept']);p=1/(1+np.exp(-np.clip(linear,-700,700)))
 return p,p>=.5
