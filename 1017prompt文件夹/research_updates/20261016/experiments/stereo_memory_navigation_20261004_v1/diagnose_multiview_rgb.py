"""Post-prediction only, ONE true-depth patch label per source; no new candidate."""
from pathlib import Path
import json,datetime
import cv2,numpy as np
from multiview_rgb import optical_rotation
from past_multiview import measure_view
H=Path(__file__).resolve().parent

def main():
    p=json.loads((H/'multiview_rgb_protocol.json').read_text());run=json.loads((H/'multiview_rgb_predictions/manifest.json').read_text())
    started=datetime.datetime.now(datetime.timezone.utc).isoformat();assert all(r['ended_utc']<started for r in run['rows'])
    inputs={};truth={}
    manifest=json.loads((H/'transaction_development_manifest.json').read_text())
    for r in manifest['samples']:
        if r['sequence']==p['sequence']:
            f=int(Path(r['rgb_file']).stem.split('_')[1]);inputs[f]=np.load(H/'multiview_rgb_inputs'/f'frame_{f:03d}.npz');truth[f]=np.load(r['truth_file'])
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    past=np.load(H/'past_multiview_measurements.npz');arrays={};rows=[]
    offsets=np.array([(dy,dx) for dy in range(-2,3) for dx in range(-2,3)])
    for r in run['rows']:
        f=r['frame'];pre=f'frame_{f:03d}__';points=past[pre+'source_world_m'];yx=past[pre+'source_pixels_yx'];yy,xx=yx.T
        rp=inputs[f]['left_pose'];rr=optical_rotation(rp);z=((points-rp[:3,3])@np.linalg.inv(rr).T)[:,2]
        qy=yy[:,None]+offsets[:,0];qx=xx[:,None]+offsets[:,1]
        refgray=cv2.cvtColor(inputs[f]['left_rgb'],cv2.COLOR_RGB2GRAY).astype(float)
        refinside=(qx>=0)&(qx<640)&(qy>=0)&(qy<480);sy=np.clip(qy,0,479);sx=np.clip(qx,0,639)
        a=refgray[sy,sx];refstd=a.std(axis=1);refvalid=refinside.all(axis=1)&~masks['left'][sy,sx].any(axis=1)&(refstd>=1)
        rays=np.stack([(qx-p['cx'])/p['focal_px'],(qy-p['cy'])/p['focal_px'],np.ones_like(qx)],axis=-1)
        wp=(rays*z[:,None,None])@rr.T+rp[:3,3]
        anyvisible=np.zeros(len(z),bool);anymatch=np.zeros(len(z),bool);source_rows=[]
        for source in r['sources']:
            sf=source['frame'];eye=source['eye'];sp=inputs[sf][eye+'_pose'];sr=optical_rotation(sp)
            optical=(wp-sp[:3,3])@np.linalg.inv(sr).T
            with np.errstate(divide='ignore',invalid='ignore'):
                x=optical[:,:,0]/optical[:,:,2]*p['focal_px']+p['cx'];y=optical[:,:,1]/optical[:,:,2]*p['focal_px']+p['cy']
            finite=np.isfinite(x)&np.isfinite(y)&(optical[:,:,2]>0);x=np.where(finite,x,-1);y=np.where(finite,y,-1)
            inside=finite&(x>=0)&(x<639)&(y>=0)&(y<479)
            x0=np.floor(np.clip(x,0,638)).astype(int);y0=np.floor(np.clip(y,0,478)).astype(int);sm=masks[eye]
            valid=inside&~(sm[y0,x0]|sm[y0+1,x0]|sm[y0,x0+1]|sm[y0+1,x0+1])
            b=cv2.remap(cv2.cvtColor(inputs[sf][eye+'_rgb'],cv2.COLOR_RGB2GRAY).astype(np.float32),x.astype(np.float32),y.astype(np.float32),cv2.INTER_LINEAR).astype(float)
            bs=b.std(axis=1);ac=a-a.mean(axis=1,keepdims=True);bc=b-b.mean(axis=1,keepdims=True)
            denom=refstd*bs;cost=1-np.clip(np.divide(np.mean(ac*bc,axis=1),denom,out=np.zeros_like(denom),where=denom>0),-1,1)
            r1=points-rp[:3,3];r2=points-sp[:3,3]
            angle=np.degrees(np.arctan2(np.linalg.norm(np.cross(r1,r2),axis=1),np.sum(r1*r2,axis=1)))
            geom=measure_view(points,sp,truth[sf][eye+'_depth_m'],sm,p['focal_px'],p['cx'],p['cy'],.01)['visible']
            geom &= past[pre+'current_left']
            supported=refvalid&valid.all(axis=1)&(bs>=1)&(angle>=.5)
            considered=geom&supported;match=considered&(cost<=.2)
            anyvisible|=geom;anymatch|=match
            arrays[pre+f'source_{sf:03d}_{eye}__cost_at_true_depth']=cost.astype(np.float32)
            arrays[pre+f'source_{sf:03d}_{eye}__visible_supported']=considered
            def percentile(a,q):return float(np.percentile(a,q)) if len(a) else None
            source_rows.append(dict(frame=sf,eye=eye,true_visible_n=int(geom.sum()),visible_full_patch_supported_n=int(considered.sum()),
                true_depth_cost_le_0p2_n=int(match.sum()),true_depth_cost_median=percentile(cost[considered],50),
                true_depth_cost_p90=percentile(cost[considered],90),ref_low_texture_visible_n=int((geom&(refstd<1)).sum()),
                source_low_texture_visible_n=int((geom&(bs<1)).sum()),parallax_rejected_visible_n=int((geom&(angle<.5)).sum()),
                footprint_rejected_visible_n=int((geom&~(refinside.all(axis=1)&valid.all(axis=1)) ).sum())))
        rows.append(dict(frame=f,source_rows=source_rows,any_true_visible_n=int(anyvisible.sum()),any_true_depth_patch_cost_le_0p2_n=int(anymatch.sum())))
        arrays[pre+'any_true_depth_patch_cost_le_0p2']=anymatch;print('TRUE_DEPTH_DIAGNOSTIC',rows[-1],flush=True)
    np.savez_compressed(H/'multiview_rgb_true_depth_diagnostic.npz',**arrays)
    (H/'multiview_rgb_true_depth_diagnostic.json').write_text(json.dumps(dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        role='offline label-conditioned photometric diagnosis AFTER frozen predictions, no true-depth input to candidate/no candidate rerun',
        limitations='true point visibility uses full 2x2 labels, not all 25 patch depths; true frontoparallel plane warp can still misrepresent curved surface; one cost at exact true depth does not imply unique available metric estimate',rows=rows),indent=2)+'\n')
if __name__=='__main__':main()
