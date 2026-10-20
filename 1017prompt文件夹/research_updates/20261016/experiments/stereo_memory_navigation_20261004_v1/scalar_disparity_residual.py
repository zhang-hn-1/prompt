"""One frozen scalar MODEL correction. Raw pair identities are not re-matched.
Never asserts corrected LR consistency, observed free or model admission.
"""
import numpy as np
F=584.8888888888889;B=.6
def corrected_world(x,y,disparity,bias,T):
 if not np.isfinite(bias):raise ValueError('finite frozen model')
 x,y,d=np.asarray(x,float),np.asarray(y,float),np.asarray(disparity,float)+bias
 if x.shape!=y.shape or x.shape!=d.shape or x.ndim!=1 or np.asarray(T).shape!=(4,4) or not np.isfinite(T).all():raise ValueError('aligned queries and current estimated pose')
 with np.errstate(divide='ignore',invalid='ignore'):z=F*B/d
 valid=np.isfinite(x)&np.isfinite(y)&np.isfinite(z)&(d>0)&(z>=.5)&(z<=3)
 rays=np.column_stack([(x-319.5)/F,(y-239.5)/F,np.ones(len(x))]);w=np.full((len(x),3),np.nan);w[valid]=T[:3,3]+(rays[valid]*z[valid,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 return w,valid
