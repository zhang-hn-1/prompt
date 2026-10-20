"""Exactly one E0-only balanced logistic action fit, existing SciPy only."""
from common import *
from policy_model import AdmissionActionModel
from scipy.optimize import minimize
from scipy.special import expit
import os

def objective(theta,Z,Y,sw):
    logits=Z@theta[:-1]+theta[-1]
    loss=float(np.sum(sw*(np.logaddexp(0.,logits)-Y*logits))+.5*np.dot(theta[:-1],theta[:-1]))
    residual=sw*(expit(logits)-Y)
    gradient=np.r_[Z.T@residual+theta[:-1],residual.sum()]
    return loss,gradient

def main():
    cfg=load(D/'protocol.json');assert not (D/'fit_started.json').exists()
    for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
    for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
    assert load(cfg['consumer_result'])['necessary_passed']
    assert load(cfg['frontend_result'])['decision']=='FULL470_CAUSAL_FEATURE_INTERFACE_BYTE_EQUIVALENT_NOT_FIT_ADMISSION'
    fs=load(cfg['feature_seal']);actions=load(cfg['action_seal']);X=[];Y=[];index=[]
    for f,a in zip(fs['rows'],actions['rows']):
        assert (f['sequence'],f['frame'])==(a['sequence'],a['frame'])
        if f['sequence']!='E0':continue
        assert sha(f['feature_path'])==f['feature_sha256'] and sha(a['path'])==a['sha256']
        with np.load(f['feature_path']) as raw,np.load(a['path']) as teacher:
            assert np.array_equal(raw['cells'],teacher['cells'])
            x=raw['features'];y=teacher['teacher_action']
            assert x.shape==(len(y),24) and np.isfinite(x).all() and np.isin(y,[0,1]).all()
            X.append(x);Y.append(y);index.append(dict(frame=f['frame'],rows=len(y),WITHHOLD_NEW=int(y.sum()),feature_sha256=f['feature_sha256'],action_sha256=a['sha256']))
    X=np.concatenate(X);Y=np.concatenate(Y);assert X.shape==(2038,24) and int(Y.sum())==156
    mean=X.mean(0);scale=X.std(0);scale[scale<1e-6]=1.;Z=(X-mean)/scale
    sw=np.where(Y==1,len(Y)/(2*int(Y.sum())),len(Y)/(2*int((Y==0).sum())))
    # Fixed analytic gradient fixture, no data optimization or second fit.
    zz=np.arange(72,dtype=float).reshape(3,24)/70;yy=np.array([0.,1.,0.]);ww=np.array([.75,1.5,.75]);tt=np.linspace(-.2,.2,25)
    value,grad=objective(tt,zz,yy,ww);num=[]
    for j in range(25):
        e=np.zeros(25);e[j]=1e-6
        num.append((objective(tt+e,zz,yy,ww)[0]-objective(tt-e,zz,yy,ww)[0])/2e-6)
    assert np.max(abs(grad-num))<1e-8
    put(D/'fit_dataset_receipt.json',dict(utc=now(),rows=len(Y),HOLD_BASELINE=int((Y==0).sum()),WITHHOLD_NEW=int(Y.sum()),index=index,fit_scene='E0 only; E1A/B labels not used in fit',feature_names=fs['feature_names'],gradient_fixture_max_error=float(np.max(abs(grad-num)))))
    put(D/'fit_started.json',dict(utc=now(),pid=os.getpid(),fit_used=1,fit_cap=1,actual_ML_fit=True))
    result=minimize(objective,np.zeros(25),args=(Z,Y,sw),method='L-BFGS-B',jac=True,
        options=dict(maxiter=200,gtol=1e-4,ftol=2.220446049250313e-9,maxls=20))
    assert np.isfinite(result.x).all() and np.isfinite(result.fun)
    weights=dict(mean=mean.tolist(),scale=scale.tolist(),coef=result.x[:-1].tolist(),intercept=float(result.x[-1]),
        score_meaning='P(WITHHOLD_NEW teacher action) under a fitted logistic model; not calibrated physical risk',
        threshold=.95,classes=['HOLD_BASELINE','WITHHOLD_NEW'],feature_channels=24,source_calls=0)
    runtime=AdmissionActionModel(weights)
    assert np.max(abs(runtime.score(X)-expit(Z@result.x[:-1]+result.x[-1])))<1e-14
    put(D/'weights.json',weights)
    put(D/'weight_seal.json',dict(utc=now(),weight_sha256=sha(D/'weights.json'),protocol_sha256=sha(D/'protocol.json'),
        fit_dataset_receipt_sha256=sha(D/'fit_dataset_receipt.json'),fit_sequences=['E0'],dev_sequences=['E1B','E1A'],
        E1A_B_one_seen_development_scene=True,heldout_acquired=False,heldout_pixels_or_teacher_opened=False,
        optimizer_iterations=int(result.nit),objective_evaluations=int(result.nfev),final_objective=float(result.fun),
        fit_converged=bool(result.success),optimizer_status=int(result.status),optimizer_message=str(result.message),
        fit_used=1,fit_source_rows=2038,fit_consistency_score_rows=2038))
    print('ONE_ACTION_MODEL_SEALED',int(result.nit),'converged',bool(result.success),flush=True)
if __name__=='__main__':main()
