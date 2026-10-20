"""Postseal conditional coverage attribution, labels only offline, no source changes."""
from paths import *
from source import NAMES,RAYS,AX,F
import numpy as np,json,hashlib,time
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    cfg=json.loads((D/'protocol.json').read_text());diag=json.loads((D/'overlap_protocol.json').read_text());assert sha(Path(__file__))==diag['source_sha256']
    seal=json.loads((D/'prediction_seal.json').read_text());assert seal['complete'];rows=[]
    totals={n:{k:0 for k in ['target','partner_FOV_range_CAD','partner_depth_compatible','source_valid','source_valid_in_shared','depth_error_le15cm','shared_depth_error_le15cm']} for n in NAMES}
    for r in seal['rows']:
        ip=Path(cfg['data_root'])/'frames'/r['frame'];pp=D/'e0_predictions'/r['frame'];assert sha(ip)==r['input_sha256'] and sha(pp)==r['prediction_sha256']
        with np.load(ip) as a,np.load(pp) as p:
            for n in NAMES:
                partner={'front_left':'front_right','front_right':'front_left','rear_left':'rear_right','rear_right':'rear_left'}[n]
                z=a[n+'_depth_label'];T=a[n+'_camera_usd_world_label'];P=a[partner+'_camera_usd_world_label'];ok=np.isfinite(z)&(z>=.5)&(z<=3)&~p[n+'_CAD']
                ids=np.flatnonzero(ok);zz=z.ravel()[ids];w=(RAYS.reshape(-1,3)[ids]*zz[:,None])@(T[:3,:3]@AX).T+T[:3,3]
                target=(np.linalg.norm(w-a['root_position_physics_label'],axis=1)>=.55)&(w[:,2]>.05)&(w[:,2]<=1.5);ids=ids[target];zz=zz[target];w=w[target]
                c=(w-P[:3,3])@(P[:3,:3]@AX);u=F*c[:,0]/c[:,2]+319.5;v=F*c[:,1]/c[:,2]+239.5
                inside=(u>=0)&(u<=639)&(v>=0)&(v<=479)&(c[:,2]>=.5)&(c[:,2]<=3)
                iu=np.rint(np.clip(u,0,639)).astype(int);iv=np.rint(np.clip(v,0,479)).astype(int)
                inside &= ~p[partner+'_CAD'][iv,iu]
                partnerz=a[partner+'_depth_label'][iv,iu]
                shared=inside&np.isfinite(partnerz)&(abs(partnerz-c[:,2])<=.02)
                pd=p[n+'_depth'].ravel()[ids];valid=p[n+'_valid'].ravel()[ids]&np.isfinite(pd)&(pd>=.5)&(pd<=3);good=valid&(abs(pd-zz)<=.15)
                row=dict(frame=r['frame'],eye=n,target=len(ids),partner_FOV_range_CAD=int(inside.sum()),partner_depth_compatible=int(shared.sum()),source_valid=int(valid.sum()),source_valid_in_shared=int((valid&shared).sum()),depth_error_le15cm=int(good.sum()),shared_depth_error_le15cm=int((good&shared).sum()))
                rows.append(row)
                for k in totals[n]:totals[n][k]+=row[k]
    assert sum(v['target'] for v in totals.values())==4476747
    (D/'overlap_result.json').write_text(json.dumps(dict(complete=True,per_eye=totals,rows=rows,scope=diag['scope'],source_changes=0,fit=0,physics=0),indent=2)+'\n');print('OVERLAP_RESULT',totals,flush=True)
if __name__=='__main__':main()
