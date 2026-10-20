"""Single immutable affine head on frozen semantic probabilities and RGB pixels."""
import numpy as np

def fold_affine(mean,scale,coefficient,intercept):
    mean=np.asarray(mean,float);scale=np.asarray(scale,float);coef=np.asarray(coefficient,float);b=float(intercept)
    if mean.ndim!=1 or mean.shape!=scale.shape or mean.shape!=coef.shape or not np.isfinite(mean).all() or not np.isfinite(scale).all() or not np.isfinite(coef).all() or not np.isfinite(b) or (scale<=0).any():raise ValueError('finite matching affine parameters and positive scales required')
    weight=coef/scale;return dict(weight=weight.copy(),bias=b-float(np.dot(weight,mean)))
def apply_head(packet,head):
    if set(packet)!={'features'}:raise ValueError('feature-only inference packet required')
    x=np.asarray(packet['features']);weight=np.asarray(head['weight'],float);bias=float(head['bias'])
    if x.ndim!=2 or x.shape[1]!=len(weight) or not np.isfinite(x).all() or not np.isfinite(weight).all() or not np.isfinite(bias):raise ValueError('finite matching feature matrix and frozen head required')
    logit=x@weight+bias;return 1/(1+np.exp(-np.clip(logit,-80,80)))
def features_at_pixels(probabilities_native,rgb,yy,xx):
    prob=np.asarray(probabilities_native);rgb=np.asarray(rgb);y=np.asarray(yy);x=np.asarray(xx)
    if prob.ndim!=3 or prob.shape[0]!=150 or rgb.shape!=(*prob.shape[1:],3) or y.shape!=x.shape or y.ndim!=1:raise ValueError('150 semantic probabilities plus matching RGB/pixel index vectors required')
    return np.concatenate([prob[:,y,x].T.astype(np.float32),rgb[y,x].astype(np.float32)/255],axis=1)

def balanced_logistic_objective(theta,features,labels,sample_weight):
    from scipy.special import expit
    theta=np.asarray(theta,float);x=np.asarray(features);y=np.asarray(labels);weight=np.asarray(sample_weight)
    z=x@theta[:-1]+theta[-1];residual=weight*(expit(z)-y)
    loss=float(np.sum(weight*(np.logaddexp(0,z)-y*z))+.5*np.dot(theta[:-1],theta[:-1]))
    gradient=np.r_[x.T@residual+theta[:-1],residual.sum()]
    return loss,gradient
