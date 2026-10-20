"""One frozen low-order next-frame image-flow model; no environment safety output."""
import numpy as np

def causal_features(sensor,xy,prior_flow,prior_valid,eye,current_time,prior_end_time):
 if not np.isfinite([current_time,prior_end_time]).all() or abs(current_time-prior_end_time)>1e-6:raise ValueError('past flow must end at current finite clock')
 s=np.asarray(sensor,float);q=np.asarray(xy,float);p=np.asarray(prior_flow,float);v=np.asarray(prior_valid,bool)
 if s.shape!=(14,) or q.ndim!=2 or q.shape[1]!=2 or p.shape!=q.shape or v.shape!=(len(q),) or eye not in (0,1):raise ValueError('causal input shapes')
 if not np.isfinite(s).all() or not np.isfinite(q).all() or not np.isfinite(p[v]).all():raise ValueError('finite available inputs')
 b=np.where(v[:,None],p,0.);z=(q-np.array([319.5,239.5]))/np.array([320.,240.]);products=np.column_stack([z[:,0]**2,z[:,0]*z[:,1],z[:,1]**2])
 x=np.column_stack([np.broadcast_to(s,(len(q),14)),z,products,b,v.astype(float),np.full(len(q),float(eye))])
 return x,b

def fit_once(x,y,baseline,weights):
 x=np.asarray(x,float);y=np.asarray(y,float);b=np.asarray(baseline,float);w=np.asarray(weights,float)
 if x.ndim!=2 or x.shape[1]!=23 or y.shape!=b.shape or y.shape!=(len(x),2) or w.shape!=(len(x),) or not np.isfinite(x).all() or not np.isfinite(y).all() or not np.isfinite(b).all() or not np.isfinite(w).all() or np.any(w<=0):raise ValueError('finite historical fit arrays')
 mean=np.average(x,axis=0,weights=w);scale=np.maximum(np.sqrt(np.average((x-mean)**2,axis=0,weights=w)),1.)
 z=np.column_stack([np.ones(len(x)),(x-mean)/scale]);gram=z.T@(w[:,None]*z);penalty=np.eye(24);penalty[0,0]=0
 coef=np.linalg.solve(gram+penalty,z.T@(w[:,None]*(y-b)))
 return dict(mean=mean,scale=scale,coef=coef)

def predict(model,x,baseline):
 x=np.asarray(x,float);b=np.asarray(baseline,float)
 if x.ndim!=2 or x.shape[1]!=23 or b.shape!=(len(x),2) or not np.isfinite(x).all() or not np.isfinite(b).all():raise ValueError('finite causal predict input')
 return b+np.column_stack([np.ones(len(x)),(x-model['mean'])/model['scale']])@model['coef']
