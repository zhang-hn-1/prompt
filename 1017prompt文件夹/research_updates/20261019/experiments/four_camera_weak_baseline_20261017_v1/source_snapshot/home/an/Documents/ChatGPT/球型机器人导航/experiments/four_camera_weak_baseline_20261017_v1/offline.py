from paths import *
from source import *
import json,hashlib,sys
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    cfg=json.loads((D/'protocol.json').read_text()); run=Path(cfg['data_root']);seal=json.loads((run/'capture_seal.json').read_text())
    for p,h in cfg['source_sha256'].items():assert sha(p)==h,p
    out=D/'e0_predictions';out.mkdir(exist_ok=False); model=FourSGBM();rows=[]
    for i,row in enumerate(seal['rows']):
        path=run/'frames'/row['frame'];assert sha(path)==row['sha256']
        with np.load(path) as a:
            # Explicit whitelist: labels never accessed by prediction.
            rgb={n:a[n+'_rgb'] for n in NAMES};q=a['joints_current'];t=float(a['source_time_s'])
            body=a['front_left_camera_usd_world_estimated']@np.linalg.inv(LOCAL[0])
            pred,wall=model.predict(rgb,q);records,save=points(pred,body,i,t)
            for n in NAMES:
                for k in ('depth','valid','disparity','CAD'):save[n+'_'+k]=pred[n][k]
            save.update(body_estimated=body,time_s=t)
            dest=out/row['frame'];np.savez_compressed(dest,**save)
            rows.append(dict(frame=row['frame'],time_s=t,input_sha256=sha(path),prediction_sha256=sha(dest),generation_s=wall,eyes={n:dict(LR_valid_pixels=int(pred[n]['valid'].sum()),range_pixels=int((pred[n]['valid']&(pred[n]['depth']>=.5)&(pred[n]['depth']<=3)).sum()),risk_points=len(save[n+'_world'])) for n in NAMES}))
            if i%10==0:print(i,len(seal['rows']),rows[-1],flush=True)
    (D/'prediction_seal.json').write_text(json.dumps(dict(complete=True,rows=rows,physics=0,fit=0,GT_decision=False,source_sha256=cfg['source_sha256']),indent=2)+'\n')
    print('SEALED',len(rows),flush=True)
if __name__=='__main__':main()
