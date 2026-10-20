"""Exact current/past whitelist feature generation; teacher loaded only in later process."""
from common import *
from consumer import EntryState,interface_checks
from scipy.ndimage import distance_transform_edt
import datetime

def main():
    cfg=load(D/'protocol.json');assert not (D/'feature_seal.json').exists()
    for f,h in cfg['feature_metadata_sha256'].items():assert sha(f)==h,f
    checks=interface_checks();allrows=[];totalpoints=0;filtered=0;mockmatches=0
    for seq in cfg['sequences']:
        cap=load(seq['capture_seal']);src=load(seq['source_seal']);native=load(seq['native_imu']);assert cap['complete'] and src['complete'];history={};lifetime={};origin=None;last=-np.inf;engine=EntryState();out=D/'features'/seq['name'];out.mkdir(parents=True)
        for i in range(seq['active_frames']):
            s=src['rows'][i];c=cap['rows'][i];fn=c['frame'];raw=Path(seq['capture_root'])/'frames'/fn;pf=Path(seq['prediction_root'])/fn
            assert sha(raw)==s['input_sha256']==c['sha256'] and sha(pf)==s['prediction_sha256']
            t=float(s['time_s']);assert t>last or i==0;assert abs(float(native[c['imu_index']]['time_s'])-t)<1e-8 and native[c['imu_index']]['qualified'];last=t
            with np.load(pf) as p,np.load(raw) as a:
                body=p['body_estimated'];assert np.isfinite(body).all() and np.allclose(body,np.array(native[c['imu_index']]['estimated_base_world']),atol=1e-6,rtol=1e-6)
                assert abs(float(p['time_s'])-t)<1e-8
                if origin is None:origin=body[:2,3]-20
                # Explicit raw-frame access: ONLY RGB is accessed. No renderer depth/pose or scene data.
                rgb=[a[n+'_rgb'] for n in NAMES]
                assert all(x.shape==(480,640,3) and x.dtype==np.uint8 for x in rgb)
                vv=[];ww=[];ee=[];qq=[];pp=[];cc=[];total=0
                for e,n in enumerate(NAMES):
                    w=p[n+'_world'];q=p[n+'_query_pixel'];partner=p[n+'_partner_pixel'];z=p[n+'_depth'].reshape(-1)[q];disp=abs(p[n+'_disparity'].reshape(-1)[q]);assert len(w)==len(q)==len(partner)
                    assert np.isfinite(w).all() and np.isfinite(z).all() and np.all((z>=.5)&(z<=3));total+=len(w)
                    gray=rgb[e].astype(float).mean(2);gy,gx=np.gradient(gray);grad=np.hypot(gx,gy).reshape(-1)[q];mad=abs(rgb[e].reshape(-1,3)[q].astype(float)-rgb[PARTNER[e]].reshape(-1,3)[partner].astype(float)).mean(1);cad=distance_transform_edt(~p[n+'_CAD']).reshape(-1)[q]
                    cell=np.rint((w[:,:2]-origin)/.1).astype(np.int64);inside=(cell>=0).all(1)&(cell<400).all(1);filtered+=int((~inside).sum())
                    # Numeric channels are local/relative or estimatedheight, NEVER globalXY/frame/scene IDs.
                    v=np.c_[z,w[:,2],np.linalg.norm(w-body[:3,3],axis=1),disp,gray.reshape(-1)[q],grad,mad,cad]
                    vv.append(v[inside]);ww.append(w[inside]);ee.append(np.full(inside.sum(),e,int));qq.append(q[inside]);pp.append(partner[inside]);cc.append(cell[inside])
                totalpoints+=total;vals=np.concatenate(vv);world=np.concatenate(ww);eye=np.concatenate(ee);uids=np.concatenate(qq);partner=np.concatenate(pp);cells=np.concatenate(cc);keys,inverse=np.unique(cells,axis=0,return_inverse=True)
                X=[];eligible=[]
                for j,keyarr in enumerate(keys):
                    key=tuple(keyarr);ix=inverse==j;v=vals[ix];xyz=world[ix];eyes=set(map(int,eye[ix]));lifetime[key]=lifetime.get(key,0)+1
                    h=[x for x in history.get(key,[]) if t-x['time']<=10.];old=h.copy();cur=dict(time=t,z=float(v[:,0].mean()),height=float(v[:,1].mean()),gray=float(v[:,4].mean()),eyes=eyes,body=body[:3,3].copy());h.append(cur);history[key]=h
                    recenteyes=set().union(*(x['eyes'] for x in h));span=t-h[0]['time'];motion=max(np.linalg.norm(x['body']-body[:3,3]) for x in h)
                    age=min(10.,t-old[-1]['time'])/10 if old else 1.
                    f=[np.log1p(len(v)),len(eyes)/4,np.log1p(lifetime[key]),span/10,v[:,0].mean()/3,v[:,0].std()/3,v[:,1].mean()/1.5,v[:,1].std()/1.5,v[:,2].mean()/3,v[:,3].mean()/448,v[:,3].std()/448,v[:,4].mean()/255,v[:,4].std()/255,v[:,5].mean()/255,v[:,6].mean()/255,v[:,7].mean()/640,np.linalg.norm(xyz[:,:2].std(0))/.1,np.std([x['height'] for x in h])/1.5,motion/3,np.std([x['z'] for x in h])/3,len(recenteyes)/4,np.std([x['gray'] for x in h])/255,age,len({e//2 for e in eyes})/2]
                    assert len(f)==len(FEATURES) and np.isfinite(f).all();X.append(f);eligible.append(lifetime[key]>=2)
                X=np.array(X,float).reshape(-1,len(FEATURES));labels,clock=engine.update(i,t,t,keys,np.ones(len(keys)),i,t)
                # Current fixedB source equivalence check, no teacher. All-admit is an engineering mock.
                if seq['B_map_root']:
                    mf=Path(seq['B_map_root'])/f'frame_{i if seq["name"]=="E0" else 5*i:03d}.npz'
                    assert sha(mf)==seq['B_map_sha256'][str(mf)]
                    with np.load(mf) as m:
                        expected=m['B_labels'] if seq['name']=='E0' else m['labels'];assert np.array_equal(labels,expected),(seq['name'],i)
                        assert np.allclose(origin,m['origin'],atol=1e-10,rtol=0)
                    mockmatches+=1
                assert not np.any(labels==0);dest=out/fn
                np.savez_compressed(dest,features=X,cells=keys,origin=origin,resolution=.1,time_s=t,promotion_eligible=np.array(eligible,bool),source_eye=eye,source_query_uid=uids,source_partner_uid=partner,source_cell_row=inverse,original_points=total,in_bounds_points=len(vals),body_estimated=body,mock_all_admit_labels=labels)
                allrows.append(dict(sequence=seq['name'],group=seq['group'],role=seq['role'],frame=i,time_s=t,feature_path=str(dest),feature_sha256=sha(dest),input_rgb_frame=str(raw),input_sha256=sha(raw),prediction_path=str(pf),prediction_sha256=sha(pf),source_points=total,in_bounds_points=len(vals),cells=len(keys),eligible=int(np.sum(eligible)),max_feature_source_time=t,teacher_read=False))
            if i%40==0:print('FEATURES',seq['name'],i,len(keys),flush=True)
    assert len(allrows)==470
    write(D/'feature_seal.json',dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=allrows,feature_names=FEATURES,original_source_points=totalpoints,outside_consumer_grid_points=filtered,all_admit_mock_B_maps_exact=mockmatches,checks=checks,teacher_read=False,fit=0,source_prediction_regeneration=0,protocol_sha256=sha(D/'protocol.json')))
    print('FEATURES_SEALED',len(allrows),totalpoints,filtered,mockmatches,flush=True)
if __name__=='__main__':main()
