"""Separate ideal label preparation, depth generation, then GT-only scoring."""
from pathlib import Path
import sys, json, hashlib, time, datetime
from collections import Counter, defaultdict
import numpy as np
from scipy.spatial import cKDTree
from geometry import AX, solve_shape, ray_depth

D=Path(__file__).resolve().parent;W=D.parents[1]
NAMES=['front_left','front_right','rear_left','rear_right']
CFG=json.loads((D/'protocol.json').read_text());G=CFG['geometry']
H=W/'experiments/contact_boundary_source_20261016_v1'
ROOT=W/CFG['data_root']
sys.path.insert(0,str(W/'experiments/dual_monocular_semantic_bev_20261015_v1'))
import projection as consumer
from semantic_projection import project_semantic
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
write=lambda p,v:p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False,default=lambda x:x.item() if isinstance(x,np.generic) else (_ for _ in ()).throw(TypeError(type(x).__name__)))+'\n')

def project(points,T,F):
    optical=(points-T[:3,3])@(T[:3,:3]@AX)
    uv=optical[:,:2]/np.maximum(optical[:,2,None],1e-15)*F+[319.5,239.5]
    return uv,optical

def visible(points,T,F,z,mask):
    uv,opt=project(points,T,F);pix=np.rint(uv).astype(int)
    valid=(opt[:,2]>=.5)&(opt[:,2]<=3)&(points[:,2]>.001)&(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)
    seen=np.zeros(len(points),bool)
    for dx in [-1,0,1]:
        for dy in [-1,0,1]:
            xx=pix[:,0]+dx;yy=pix[:,1]+dy
            q=np.flatnonzero(valid&(xx>=0)&(xx<640)&(yy>=0)&(yy<480))
            if not len(q):continue
            dd=z[yy[q],xx[q]]
            seen[q]|=np.isfinite(dd)&~mask[yy[q],xx[q]]&(abs(dd-opt[q,2])<=.02)
    return seen,uv,opt

def contour_labels(ob,T,F,z,mask):
    c=np.asarray(ob['center']);radius=ob['radius'];R=T[:3,:3]@AX
    lines=[];uvs=[];rim_rays=[[],[]]
    if ob['type']=='Sphere':
        v=c-T[:3,3];dist=np.linalg.norm(v)
        if dist<=radius:return np.empty((0,3)),np.empty((0,2)),rim_rays
        axis=v/dist;unit=np.eye(3)[np.argmin(abs(axis))];e1=np.cross(axis,unit);e1/=np.linalg.norm(e1);e2=np.cross(axis,e1)
        theta=np.arange(128)*2*np.pi/128
        rad=radius*np.sqrt(1-radius**2/dist**2)
        points=T[:3,3]+(1-radius**2/dist**2)*v+rad*(np.cos(theta)[:,None]*e1+np.sin(theta)[:,None]*e2)
        tangent=rad*(-np.sin(theta)[:,None]*e1+np.cos(theta)[:,None]*e2)
        seen,uv,opt=visible(points,T,F,z,mask)
        keep=seen&np.roll(seen,1)&np.roll(seen,-1)
        line=np.cross(opt,tangent@R);line/=np.maximum(np.linalg.norm(line,axis=1,keepdims=True),1e-15)
        line[(line@((c-T[:3,3])@R))<0]*=-1
        return line[keep],uv[keep],rim_rays
    v=c[:2]-T[:2,3];dist=np.linalg.norm(v)
    if dist<=radius:return np.empty((0,3)),np.empty((0,2)),rim_rays
    axis=v/dist;perp=np.array([-axis[1],axis[0]])
    for sign in [-1,1]:
        n=radius/dist*axis+sign*np.sqrt(1-radius**2/dist**2)*perp
        xy=c[:2]-radius*n
        heights=np.linspace(c[2]-ob['height']/2,c[2]+ob['height']/2,17)
        points=np.c_[np.tile(xy,(17,1)),heights]
        seen,uv,_=visible(points,T,F,z,mask)
        if np.count_nonzero(seen)>=3:
            line=np.r_[n,0.]@R
            lines.append(line);uvs.append(np.mean(uv[seen],axis=0))
    theta=np.arange(64)*2*np.pi/64
    for side,cap in enumerate([c[2]-ob['height']/2,c[2]+ob['height']/2]):
        points=np.c_[c[0]+radius*np.cos(theta),c[1]+radius*np.sin(theta),np.full(64,cap)]
        seen,uv,opt=visible(points,T,F,z,mask)
        # Ideal visible cap-edge/rim labels, not renderer depth sent to solver.
        rim_rays[side]=opt[seen]/opt[seen,2,None]
    return np.asarray(lines).reshape(-1,3),np.asarray(uvs).reshape(-1,2),rim_rays

def contract():
    cfg=G
    R=np.array([[0.,0.,1.],[1.,0.,0.],[0.,-1.,0.]])
    cc=np.array([2.,.2,.7]);radius=.3
    for family in [0,1]:
        records=[]
        for t in [np.array([0.,.3,.4]),np.array([0.,-.3,.4])]:
            T=np.eye(4);T[:3,:3]=R@AX;T[:3,3]=t
            if family==0:
                v=cc-t;dist=np.linalg.norm(v);axis=v/dist;unit=np.eye(3)[np.argmin(abs(axis))];e=np.cross(axis,unit);e/=np.linalg.norm(e);f=np.cross(axis,e)
                th=np.arange(32)*2*np.pi/32
                normals=radius/dist*axis+np.sqrt(1-radius**2/dist**2)*(np.cos(th)[:,None]*e+np.sin(th)[:,None]*f)
            else:
                v=cc[:2]-t[:2];dist=np.linalg.norm(v);axis=v/dist;perp=np.array([-axis[1],axis[0]])
                normals=np.c_[[radius/dist*axis+s*np.sqrt(1-radius**2/dist**2)*perp for s in [-1,1]],np.zeros(2)]
            records.append(dict(native=T,render=T,lines=normals@R,rims=[np.empty((0,3)),np.empty((0,3))]))
        result,why=solve_shape(records,family,'native',cfg)
        assert why=='accepted' and np.max(abs(result['centre']-cc[:len(result['centre'])]))<1e-9 and abs(result['radius']-radius)<1e-9
        assert solve_shape(records[:1],family,'native',cfg)[0] is None
        assert solve_shape([records[0],records[0]],family,'native',cfg)[0] is None
    T=np.eye(4);T[:3,:3]=R@AX;T[:3,3]=[0,0,.7]
    pred=ray_depth(np.array([[0.,0.,1.]]),T,dict(family=0,centre=np.array([2.,0.,.7]),radius=.3))
    assert abs(pred[0]-1.7)<1e-10
    write(D/'geometry_contract.json',dict(sphere_unknown_centre_radius=True,cylinder_unknown_centre_radius=True,single_view_abstains=True,zero_baseline_abstains=True,ray_intersection_exact=True))

def prepare():
    out=D/'ideal_inputs';out.mkdir(exist_ok=False)
    seal=json.loads((ROOT/'capture_seal.json').read_text());objects=json.loads((ROOT/'scene_primitives.json').read_text())['objects']
    focal=seal['initial_local_identity'][0]['optics'][0]/18*640
    yy,xx=np.indices((480,640));rays=np.stack([(xx-319.5)/focal,(yy-239.5)/focal,np.ones_like(xx)],-1).reshape(-1,3)
    rows=[];counts=Counter();start=time.monotonic()
    for rr in seal['rows']:
        p=ROOT/'frames'/rr['frame'];assert sha(p)==rr['sha256']
        with np.load(p) as a,np.load(H/'e0_recovery_qualification'/rr['frame']) as cache:
            data=dict(time_s=a['source_time_s'],focal=np.array(focal),origin=cache['origin'])
            for j,eye in enumerate(NAMES):
                T=a[eye+'_camera_usd_world_label'];N=a[eye+'_camera_usd_world_estimated'];z=a[eye+'_depth_label'];mask=cache[eye+'_CAD']
                data[eye+'_CAD']=mask
                data[eye+'_render']=T;data[eye+'_native']=N
                data[eye+'_root_native']=(N@np.linalg.inv(np.asarray(seal['initial_local_identity'][j]['local_matrix'])))[:3,3]
                data[eye+'_root_render']=a['root_position_physics_label']
                valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask
                q=np.flatnonzero(valid);world=(rays[q]*z.reshape(-1)[q,None])@(T[:3,:3]@AX).T+T[:3,3]
                k=(world[:,2]>.05)&(world[:,2]<=1.5)&(np.linalg.norm(world-a['root_position_physics_label'],axis=1)>=.55)
                q=q[k];world=world[k];ids=np.full(len(q),-1,np.int32);best=np.full(len(q),np.inf)
                lines=[];line_ids=[];uvs=[];rim=[];rim_ids=[];rim_side=[]
                for oid,ob in enumerate(objects):
                    c=np.asarray(ob['center']);r=ob['radius'];bound=np.hypot(r,ob.get('height',0)/2) if ob['type']=='Cylinder' else r
                    if np.linalg.norm(c-T[:3,3])>6+bound:continue
                    v=world-c
                    if ob['type']=='Sphere':error=abs(np.linalg.norm(v,axis=1)-r)
                    else:
                        radial=np.linalg.norm(v[:,:2],axis=1);bottom=c[2]-ob['height']/2;top=c[2]+ob['height']/2
                        side=abs(radial-r);side[(world[:,2]<bottom-.02)|(world[:,2]>top+.02)]=np.inf
                        cap=np.minimum(abs(world[:,2]-bottom),abs(world[:,2]-top));cap[radial>r+.02]=np.inf
                        error=np.minimum(side,cap)
                    good=(error<=.02)&(error<best);ids[good]=oid;best[good]=error[good]
                    lin,uv,cap=contour_labels(ob,T,focal,z,mask)
                    if len(lin):lines.extend(lin);line_ids.extend([oid]*len(lin));uvs.extend(uv)
                    for s in range(2):
                        if len(cap[s]):rim.extend(cap[s]);rim_ids.extend([oid]*len(cap[s]));rim_side.extend([s]*len(cap[s]))
                data[eye+'_q']=q.astype(np.int32);data[eye+'_ids']=ids
                data[eye+'_lines']=np.asarray(lines).reshape(-1,3);data[eye+'_line_ids']=np.asarray(line_ids,np.int32);data[eye+'_uv']=np.asarray(uvs).reshape(-1,2)
                data[eye+'_rim']=np.asarray(rim).reshape(-1,3);data[eye+'_rim_ids']=np.asarray(rim_ids,np.int32);data[eye+'_rim_side']=np.asarray(rim_side,np.int8)
                counts.update(target_pixels=len(q),identified_pixels=int((ids>=0).sum()),line_features=len(lines),rim_features=len(rim))
            data['families']=np.array([0 if ob['type']=='Sphere' else 1 for ob in objects],np.int8)
            np.savez_compressed(out/rr['frame'],**data)
            rows.append(dict(frame=rr['frame'],sha256=sha(out/rr['frame']),time_s=float(data['time_s'])))
        if len(rows)%10==0:print('PREPARE',len(rows),flush=True)
    write(D/'ideal_inputs_seal.json',dict(rows=rows,counts=counts,wall_s=time.monotonic()-start,scope='Privileged offline annotations: no dimensions/centres/depth passed to estimator; families/IDs/tangency/rim/target masks ideal. Render pose arm privileged.'))

def predict():
    assert not (D/'prediction_seal.json').exists()
    out=D/'predictions';out.mkdir(exist_ok=False)
    seal=json.loads((D/'ideal_inputs_seal.json').read_text());history=defaultdict(list);rows=[];start=time.monotonic()
    for rr in seal['rows']:
        p=D/'ideal_inputs'/rr['frame'];assert sha(p)==rr['sha256']
        with np.load(p) as a:
            now=float(a['time_s']);families=a['families'];focal=float(a['focal']);origin=a['origin']
            for oid in list(history):history[oid]=[r for r in history[oid] if now-G['history_seconds']<=r['time_s']<=now]
            for eye in NAMES:
                li=a[eye+'_line_ids']
                for oid in np.unique(li):
                    rims=[]
                    for s in range(2):rims.append(a[eye+'_rim'][(a[eye+'_rim_ids']==oid)&(a[eye+'_rim_side']==s)])
                    history[int(oid)].append(dict(time_s=now,eye=eye,lines=a[eye+'_lines'][li==oid],rims=rims,native=a[eye+'_native'],render=a[eye+'_render']))
            data=dict(origin=origin,time_s=np.array(now));counts={};maps={}
            for arm in ['render','native']:
                models={};reasons=Counter()
                for oid in np.unique(np.concatenate([a[eye+'_ids'] for eye in NAMES])):
                    if oid<0:continue
                    rec=history.get(int(oid),[])
                    assert all(r['time_s']<=now for r in rec)
                    sh,reason=solve_shape(rec,int(families[oid]),arm,G);reasons[reason]+=1
                    if sh is not None:models[int(oid)]=sh
                depths={};classes={}
                for eye in NAMES:
                    q=a[eye+'_q'];ids=a[eye+'_ids'];rays=np.c_[(q%640-319.5)/focal,(q//640-239.5)/focal,np.ones(len(q))]
                    dep=np.full(len(q),np.nan)
                    for oid,sh in models.items():
                        k=ids==oid
                        if k.any():dep[k]=ray_depth(rays[k],a[eye+'_'+arm],sh)
                    T=a[eye+'_'+arm];world=(rays*dep[:,None])@(T[:3,:3]@AX).T+T[:3,3]
                    good=np.isfinite(dep)&(dep>=.5)&(dep<=3)&(np.linalg.norm(world-a[eye+'_root_'+arm],axis=1)>=.55)
                    dep[~good]=np.nan;data[eye+'_'+arm+'_depth']=dep.astype(np.float32)
                    data[eye+'_q']=q
                    dense=np.full((480,640),np.nan,np.float32);dense.reshape(-1)[q]=dep
                    cl=np.zeros((480,640),np.uint8);cl.reshape(-1)[q]=1
                    depths[eye]=dense;classes[eye]=cl
                consumer.F=focal;pair_cells=[];conflicts=[]
                for pair in [(0,1),(2,3)]:
                    eyes=[NAMES[j] for j in pair]
                    poses={k+'_camera_usd_world':a[eye+'_'+arm] for k,eye in zip(['left','right'],eyes)}
                    poses['root_position_world_m']=a[eyes[0]+'_root_'+arm]
                    masks={k:a[eye+'_CAD'] for k,eye in zip(['left','right'],eyes)}
                    ps,cs=project_semantic([depths[e] for e in eyes],[classes[e] for e in eyes],poses,masks,origin)
                    pair_cells.append(ps[2]);conflicts.extend(cs)
                data[arm+'_cells']=np.unique(np.concatenate(pair_cells).reshape(-1,2),axis=0)
                counts[arm]=dict(solved_objects=len(models),reasons=dict(reasons),cells=len(data[arm+'_cells']),pair_conflicts=conflicts)
            np.savez_compressed(out/rr['frame'],**data)
            rows.append(dict(frame=rr['frame'],time_s=now,sha256=sha(out/rr['frame']),counts=counts))
        if len(rows)%10==0:print('PREDICT',len(rows),counts,flush=True)
    write(D/'prediction_seal.json',dict(rows=rows,wall_s=time.monotonic()-start,source_sha256={f:sha(D/f) for f in ['geometry.py','run.py','protocol.json']},scope='Depth reconstructed from ideal 2D annotations, not RGB prediction; no depth/metric object dimensions loaded by predict.'))

def metric(pred,target):
    pd=cKDTree(target*.1).query(pred*.1)[0] if len(target) and len(pred) else np.full(len(pred),np.inf)
    td=cKDTree(pred*.1).query(target*.1)[0] if len(target) and len(pred) else np.full(len(target),np.inf)
    return dict(predicted=len(pred),target=len(target),correct=int((pd<=.15).sum()),covered=int((td<=.15).sum()),precision=float(np.mean(pd<=.15)) if len(pred) else None,recall=float(np.mean(td<=.15)) if len(target) else None,position_p90_m=float(np.quantile(pd,.9)) if len(pd) and np.isfinite(pd).all() else None),pd

def score():
    seal=json.loads((D/'prediction_seal.json').read_text());rows=[];dist={k:[] for k in ['render','native']};errors={k:[] for k in dist};total=0;covered={k:0 for k in dist};cat={}
    for rr in seal['rows']:
        p=D/'predictions'/rr['frame'];assert sha(p)==rr['sha256']
        with np.load(p) as pred,np.load(H/'e0_recovery_qualification'/rr['frame']) as cache,np.load(ROOT/'frames'/rr['frame']) as a:
            target=cache['target_union'];row=dict(frame=rr['frame'],time_s=rr['time_s'],counts=rr['counts'])
            for arm in dist:
                row[arm],dd=metric(pred[arm+'_cells'],target);dist[arm].extend(dd.tolist())
                for eye in NAMES:
                    q=pred[eye+'_q'];depth=pred[eye+'_'+arm+'_depth'];truth=a[eye+'_depth_label'].reshape(-1)[q];good=np.isfinite(depth)
                    errors[arm].extend(abs(depth[good]-truth[good]).tolist());covered[arm]+=int(good.sum())
                    if arm=='render':total+=len(q)
            rows.append(row)
    agg={};gates={}
    for arm in dist:
        sums={k:sum(r[arm][k] for r in rows) for k in ['predicted','target','correct','covered']}
        sums['precision']=sums['correct']/sums['predicted'] if sums['predicted'] else None;sums['recall']=sums['covered']/sums['target']
        sums['position_p90_m']=float(np.quantile(dist[arm],.9)) if dist[arm] and np.isfinite(dist[arm]).all() else None
        sums['conditional_depth_p50_p90_p95_m']=np.quantile(errors[arm],[.5,.9,.95]).tolist() if errors[arm] else None
        sums['finite_depth_pixels']=covered[arm];sums['all_target_pixels']=total;sums['depth_pixel_coverage']=covered[arm]/total;agg[arm]=sums
        gates[arm]=dict(precision=sums['precision'] is not None and sums['precision']>=.95,recall=sums['recall']>=.8,position=sums['position_p90_m'] is not None and sums['position_p90_m']<=.15,depth=bool(errors[arm]) and np.quantile(errors[arm],.9)<=.15,every_frame_precision=all(r[arm]['predicted']==0 or r[arm]['precision']>=.95 for r in rows),every_frame_recall=all(r[arm]['target']==0 or r[arm]['recall']>=.8 for r in rows),every_frame_position=all(r[arm]['predicted']==0 or (r[arm]['position_p90_m'] is not None and r[arm]['position_p90_m']<=.3) for r in rows))
    result=dict(rows=rows,aggregate=agg,gates=gates,necessary_pass=all(all(v.values()) for v in gates.values()),frames=len(rows),eyes=len(rows)*4,physics=0,ML_fit=0,RGB_inference=0,navigation=0,geometric_parameter_estimation=True,source_admitted=False,scope='Ideal contour/family/instance/mask qualification only; no online GT or real RGB detector. Unknown/unidentified remain full target denominator. No memory/nav, no real floor false-positive rate.')
    write(D/'result.json',result);print('FINAL',json.dumps({k:result[k] for k in ['aggregate','gates','necessary_pass']},ensure_ascii=False),flush=True)

if __name__=='__main__':
    action=sys.argv[1]
    if action=='contract':contract()
    elif action=='prepare':prepare()
    elif action=='predict':predict()
    elif action=='score':score()
    else:raise ValueError(action)
