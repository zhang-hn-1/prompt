"""One fixed low-order model; fit-statistics only from whitelisted old sequences."""
import numpy as np
from scipy.special import expit
from scipy.optimize import minimize
FIT={'flat_ground_pose_D3_run_v1','offset_replication_D4_run_v1'}
def fit_historical(sequences):
 if not sequences or not set(sequences)<=FIT:raise ValueError('historical whole sequence fit whitelist')
 x=np.concatenate([np.asarray(a[0],float) for a in sequences.values()]);y=np.concatenate([np.asarray(a[1],int) for a in sequences.values()])
 if x.shape!=(len(y),14) or not np.isfinite(x).all() or not set(np.unique(y))<={0,1}:raise ValueError('finite fixed feature schema')
 if set(np.unique(y))!={0,1}:raise ValueError('both classes required; reject no fallback')
 mean=x.mean(0);scale=x.std(0);scale=np.where(scale<1e-12,1.,scale);z=(x-mean)/scale;z=np.column_stack((z,np.ones(len(z))));counts=np.bincount(y);weight=len(y)/(2*counts[y]);weight/=weight.sum()
 def objective(b):
  logits=z@b;loss=np.sum(weight*(np.logaddexp(0,logits)-y*logits))+.5*np.dot(b[:-1],b[:-1]);grad=z.T@(weight*(expit(logits)-y));grad[:-1]+=b[:-1];return loss,grad
 result=minimize(objective,np.zeros(15),method='L-BFGS-B',jac=True,options={'maxiter':200});
 if not result.success or not np.isfinite(result.x).all():raise ValueError('unique candidate optimizer rejected')
 return dict(mean=mean,scale=scale,coefficient=result.x,iterations=int(result.nit),objective=float(result.fun),fit_sequences=sorted(sequences),ridge=1.,threshold=.5,model_hypothesis=True,measured_geometry=False)
def predict(model,x):
 x=np.asarray(x,float)
 if x.ndim!=2 or x.shape[1]!=14 or not np.isfinite(x).all():raise ValueError('finite fixed prediction schema')
 return expit(((x-model['mean'])/model['scale'])@model['coefficient'][:-1]+model['coefficient'][-1])
