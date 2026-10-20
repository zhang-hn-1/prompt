"""ONE old-E1-only supervised affine fit; sealed fresh E2 labels never decoded."""
from pathlib import Path
import json,datetime,hashlib,time
import numpy as np
from scipy.optimize import minimize
from frozen_floor_features import load_frozen,native_probabilities
from floor_adapter import features_at_pixels,fold_affine,balanced_logistic_objective
from multiview_rgb import optical_rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    started=datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'floor_adapter_fit_protocol.json').read_text());seal=json.loads((H/'semantic_adapter_heldout_seal.json').read_text());assert seal['utc']<p['presealed_utc']<started
    assert sha(H/'semantic_adapter_heldout_seal.json')==p['heldout_seal_sha256'];assert not (H/'floor_adapter_fit.json').exists()
    for field in ['source_sha256','training_input_sha256','checkpoint_sha256']:
        for path,value in p[field].items():assert sha(path)==value,path
    # Integrity hashing touches opaque held-out bytes, NOT images or label statistics.
    for path,value in seal['original_artifact_sha256'].items():assert sha(path)==value,path
    out=H/'floor_adapter_training_features';out.mkdir(exist_ok=False);model,processor=load_frozen(H/'semantic_floor_model');masks=np.load(p['mask_path']);parts=[];targets=[];rows=[]
    yy,xx=np.mgrid[:480,:640];rays=np.stack([(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones((480,640))],axis=-1);grid=(yy%4==0)&(xx%4==0)
    for i in p['training_frames']:
        rgb=np.load(H/'rolling_online_run/online_inputs'/f'frame_{i:03d}.npz');truth=np.load(H/'rolling_online_run/offline_truth'/f'frame_{i:03d}.npz')
        for eye in ['left','right']:
            packet=dict(rgb=rgb[eye+'_rgb'],time_s=float(rgb['time_s']),frame=i,eye=eye);prob=native_probabilities(packet,model,processor)
            depth=truth[eye+'_depth_m'];pose=truth[eye+'_camera_usd_world']
            with np.errstate(invalid='ignore'):points=pose[:3,3]+(rays*depth[...,None])@optical_rotation(pose).T
            selected=grid&~masks[eye]&np.isfinite(depth)&(depth>=.5)&(depth<=3);ys,xs=np.where(selected);x=features_at_pixels(prob,packet['rgb'],ys,xs);y=(abs(points[ys,xs,2])<=.05).astype(np.uint8)
            file=out/f'frame_{i:03d}_{eye}.npz';np.savez_compressed(file,features=x,labels=y,pixels_yx=np.column_stack([ys,xs]));parts.append(x);targets.append(y);rows.append(dict(frame=i,eye=eye,samples=len(y),positives=int(y.sum()),file=str(file),sha256=sha(file)));print('OLD_E1_TRAIN_FEATURES',i,eye,len(y),int(y.sum()),flush=True)
    x=np.concatenate(parts).astype(np.float64);y=np.concatenate(targets).astype(np.float64);assert 0<y.sum()<len(y);mean=x.mean(axis=0);scale=x.std(axis=0);scale[scale==0]=1.;x=(x-mean)/scale;weight=np.where(y==1,len(y)/(2*y.sum()),len(y)/(2*(len(y)-y.sum())))
    clock=time.perf_counter();result=minimize(balanced_logistic_objective,np.zeros(x.shape[1]+1),args=(x,y,weight),jac=True,method='L-BFGS-B',options=dict(maxiter=200,gtol=1e-4))
    head=fold_affine(mean,scale,result.x[:-1],result.x[-1]);weights=H/'floor_adapter_parameters.npz';np.savez_compressed(weights,weight=head['weight'],bias=head['bias'],mean=mean,scale=scale,standardized_coefficient=result.x[:-1],intercept=result.x[-1])
    complete=dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),fit_count=1,training_whole_sequence='old golden E1 B0 only',training_frames=p['training_frames'],rows=rows,samples=len(y),positives=int(y.sum()),negatives=int((y==0).sum()),feature_count=x.shape[1],optimizer_converged=bool(result.success),optimizer_iterations=int(result.nit),optimizer_status=int(result.status),optimizer_message=str(result.message),objective=float(result.fun),gradient_inf_norm=float(np.max(abs(result.jac))),fit_cpu_wall_s=time.perf_counter()-clock,parameters=str(weights),parameters_sha256=sha(weights),heldout_seal_sha256=p['heldout_seal_sha256'],heldout_labels_decoded=False,candidate_admitted=False,whole_research_complete=False)
    (H/'floor_adapter_fit.json').write_text(json.dumps(complete,indent=2,allow_nan=False)+'\n');print('AFFINE_FIT_COMPLETE',result.success,result.nit,len(y),flush=True)
if __name__=='__main__':main()
