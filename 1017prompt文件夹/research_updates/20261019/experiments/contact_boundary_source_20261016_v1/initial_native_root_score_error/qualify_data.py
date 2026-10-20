"""Full-sequence GT-only target and causal native-pose consumer gates.

No fitting. All original finite visible targets, including floating and
unidentified surfaces, stay in the denominator. Geometry never enters control.
"""
from pathlib import Path
import sys,json,hashlib,time
import numpy as np
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/accepted_four_camera_geometry_20261016_v1'))
from qualify import RobotSelfMask,triangles_body,AX,NAMES
from self_mask import mask_triangles
from validate import validate

sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
arr=lambda s:np.array(sorted(s),np.int64).reshape(-1,2)
st=lambda a:set(map(tuple,a.tolist()))

def floor_shapes(objects):
    out=[]
    for j,ob in enumerate(objects):
        c=np.array(ob['center']);radius=ob['radius']
        half=ob['height']/2 if ob['type']=='Cylinder' else radius
        bottom=c[2]-half;top=c[2]+half
        category='suspended' if bottom>1e-6 else ('base_on_floor' if abs(bottom)<=1e-6 else 'floor_intersecting')
        if top<=0:category='below_floor'
        foot=None
        if bottom<=1e-6 and top>=-1e-6:
            foot=radius if ob['type']=='Cylinder' else float(np.sqrt(max(0,radius**2-c[2]**2)))
        out.append(dict(**ob,id=j,bottom=bottom,top=top,category=category,foot_radius=foot))
    return out

def is_side(world,object_id,objects):
    good=np.zeros(len(world),bool)
    for j in np.unique(object_id):
        sel=object_id==j;ob=objects[j];c=np.array(ob['center']);v=world[sel]-c
        error=abs(np.linalg.norm(v[:,:2],axis=1)-ob['radius']) if ob['type']=='Cylinder' else abs(np.linalg.norm(v,axis=1)-ob['radius'])
        good[sel]=(error<=.02)&(world[sel,2]>0.01)&(world[sel,2]<=ob['top'])&(world[sel,2]>=ob['bottom'])
    return good

def select_labels(objects,T,z,mask,F):
    R=T[:3,:3]@AX;theta=np.arange(1440)*2*np.pi/1440;points=[];ids=[]
    # Safe range bound for any original image ray at optical Z<=3 m.
    bound=3*np.sqrt(1+(320/F)**2+(240/F)**2)
    near=[]
    for j,ob in enumerate(objects):
        half=ob['height']/2 if ob['type']=='Cylinder' else ob['radius']
        radius=np.hypot(ob['radius'],half) if ob['type']=='Cylinder' else ob['radius']
        if np.linalg.norm(np.array(ob['center'])-T[:3,3])<=bound+radius:near.append(ob)
        if ob['foot_radius'] is None or np.linalg.norm(np.array(ob['center'])[:2]-T[:2,3])>bound+ob['radius']:continue
        c=np.array(ob['center']);rad=ob['foot_radius']
        points.append(np.c_[c[0]+rad*np.cos(theta),c[1]+rad*np.sin(theta),np.zeros(1440)])
        ids.extend([j]*1440)
    stats=dict(projected_samples=0,contact_supported=0,floor_supported=0,side_supported=0,two_surface_supported=0)
    if not points:return np.empty((0,2),int),np.empty(0,int),stats
    points=np.concatenate(points);ids=np.array(ids);q=(points-T[:3,3])@R
    uv=q[:,:2]/np.maximum(q[:,2,None],1e-20)*F+[319.5,239.5]
    k=(q[:,2]>=.5)&(q[:,2]<=3)&(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)
    ii=np.flatnonzero(k);pix=np.rint(uv[ii]).astype(int);stats['projected_samples']=len(ii)
    xx,yy=pix.T;keep=~mask[yy,xx];ii=ii[keep];pix=pix[keep];xx,yy=pix.T
    zc=z[yy,xx].astype(float);finite=np.isfinite(zc)&(zc>0)
    with np.errstate(invalid='ignore'):
        world=T[:3,3]+(np.c_[(pix-[319.5,239.5])/F,np.ones(len(pix))]*zc[:,None])@R.T
    contact=finite&(np.linalg.norm(world-points[ii],axis=1)<=.02)
    down=R.T@np.array([0.,0.,-1.]);delta=down[:2]-q[ii,:2]/q[ii,2,None]*down[2]
    delta/=np.maximum(np.max(abs(delta),axis=1,keepdims=True),1e-20)
    floor=np.zeros(len(pix),bool);side=floor.copy()
    for sign in [1,-1]:
        for step in [1,2,3]:
            neighbor=pix+sign*np.rint(step*delta).astype(int);u,v=neighbor.T
            valid=(u>=0)&(u<640)&(v>=0)&(v<480);sel=np.flatnonzero(valid)
            if not len(sel):continue
            u=u[sel];v=v[sel];dd=z[v,u].astype(float)
            with np.errstate(invalid='ignore'):
                w=T[:3,3]+(np.c_[(neighbor[sel]-[319.5,239.5])/F,np.ones(len(sel))]*dd[:,None])@R.T
            good=np.isfinite(dd)&(dd>0)&~mask[v,u]
            if sign==1:
                good&=abs(w[:,2])<=.01
                for ob in near:
                    if ob['foot_radius'] is not None:
                        good&=np.linalg.norm(w[:,:2]-np.array(ob['center'])[:2],axis=1)>=ob['foot_radius']-.005
                floor[sel]|=good
            else:
                good&=is_side(w,ids[ii[sel]],objects);side[sel]|=good
    valid=contact&floor&side
    labels,index=np.unique(pix[valid],axis=0,return_index=True)
    object_ids=ids[ii[valid]][index]
    stats.update(contact_supported=int(contact.sum()),floor_supported=int((contact&floor).sum()),
        side_supported=int((contact&side).sum()),two_surface_supported=int(valid.sum()))
    return labels,object_ids,stats

def project(labels,T,F,root):
    ray=np.c_[(labels-[319.5,239.5])/F,np.ones(len(labels))];direction=ray@(T[:3,:3]@AX).T
    dep=np.divide(-T[2,3],direction[:,2],out=np.full(len(labels),np.inf),where=abs(direction[:,2])>1e-12)
    with np.errstate(invalid='ignore'):world=T[:3,3]+direction*dep[:,None]
    k=(dep>=.5)&(dep<=3)&np.isfinite(world).all(1)&(np.linalg.norm(world-root,axis=1)>=.55)
    return world[k],k

def metric(prediction,target,origin):
    p=arr(prediction)*.1+origin;t=arr(target)*.1+origin
    to_t=cKDTree(t).query(p)[0] if len(t) and len(p) else np.full(len(p),np.inf)
    to_p=cKDTree(p).query(t)[0] if len(p) and len(t) else np.full(len(t),np.inf)
    return dict(prediction_cells=len(p),target_cells=len(t),correct=int((to_t<=.15).sum()),recovered=int((to_p<=.15).sum()),
        precision=float(np.mean(to_t<=.15)) if len(p) else None,recall=float(np.mean(to_p<=.15)) if len(t) else None,
        position_p90_m=float(np.quantile(to_t,.9)) if len(to_t) and np.isfinite(to_t).all() else None),to_t

def main():
    condition=sys.argv[1];name=sys.argv[2] if len(sys.argv)>2 else condition
    start=time.monotonic();cfg=json.loads((D/'protocol.json').read_text());root=D/(name+'_run')
    status=json.loads((D/(name+'_status.json')).read_text());completion=validate(condition,status,root.name)
    seal=json.loads((root/'capture_seal.json').read_text());objects=floor_shapes(json.loads((root/'scene_primitives.json').read_text())['objects'])
    local=[np.array(v['local_matrix']) for v in seal['initial_local_identity']]
    F=seal['initial_local_identity'][0]['optics'][0]/18*640
    occ=np.load(cfg['acquisitions'][condition]['occupancy']);origin=occ['global_origin_xy']
    y,x=np.indices((480,640));rays=np.stack([(x-319.5)/F,(y-239.5)/F,np.ones_like(x)],-1)
    cell=lambda w:np.unique(np.floor((w[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
    cad=RobotSelfMask();rows=[];distances={'oracle':[],'native':[]};point_errors={'oracle':[],'native':[]}
    out=D/(name+'_qualification');out.mkdir(exist_ok=False)
    for rr in seal['rows']:
        a=np.load(root/'frames'/rr['frame']);tri=triangles_body(cad,a['joints_current'])
        target=set();sets={'oracle':set(),'native':set()};arms=[];save={};categories={k:set() for k in ['base_on_floor','floor_intersecting','suspended','unidentified']}
        for j,eye in enumerate(NAMES):
            T=a[eye+'_camera_usd_world_label'];N=a[eye+'_camera_usd_world_estimated'];z=a[eye+'_depth_label'];R=T[:3,:3]@AX
            optical=(tri-local[j][:3,3])@(local[j][:3,:3]@AX);mask=mask_triangles(optical,640,480,F,319.5,239.5)
            valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask
            world=T[:3,3]+(rays[valid]*z[valid,None])@R.T
            band=(world[:,2]>.05)&(world[:,2]<=1.5)&(np.linalg.norm(world-a['root_position_physics_label'],axis=1)>=.55)
            bw=world[band];cc=cell(bw);target|=st(cc)
            # Report floating/grounded/unknown GT support independently; never trim target.
            known=np.zeros(len(bw),bool);object_visible=set()
            for ob in objects:
                c=np.array(ob['center']);bounding=np.hypot(ob['radius'],ob.get('height',0)/2) if ob['type']=='Cylinder' else ob['radius']
                if np.linalg.norm(c-T[:3,3])>6+bounding:continue
                v=bw-c
                if ob['type']=='Cylinder':
                    radial=np.linalg.norm(v[:,:2],axis=1)
                    belongs=((abs(radial-ob['radius'])<=.02)&(bw[:,2]>=ob['bottom']-.02)&(bw[:,2]<=ob['top']+.02))|((radial<=ob['radius']+.02)&((abs(bw[:,2]-ob['top'])<=.02)|(abs(bw[:,2]-ob['bottom'])<=.02)))
                else:belongs=abs(np.linalg.norm(v,axis=1)-ob['radius'])<=.02
                if belongs.any():
                    known|=belongs;object_visible.add(ob['id'])
                    key=ob['category'] if ob['category'] in categories else 'unidentified'
                    categories[key]|=st(cell(bw[belongs]))
            categories['unidentified']|=st(cell(bw[~known]))
            labels,ids,stats=select_labels(objects,T,z,mask,F)
            truepoint=T[:3,3]+(np.c_[(labels-[319.5,239.5])/F,np.ones(len(labels))]*z[labels[:,1],labels[:,0],None])@R.T
            for method,pose,root_position in [('oracle',T,a['root_position_physics_label']),('native',N,N[:3,3]-N[:3,:3]@local[j][:3,3])]:
                pred,k=project(labels,pose,F,root_position)
                cells=cell(pred);sets[method]|=st(cells)
                point_errors[method].extend(np.linalg.norm(pred[:,:2]-truepoint[k,:2],axis=1).tolist())
                save[eye+'_'+method+'_cells']=cells
            arms.append(dict(camera=eye,all_pixels=z.size,CAD_pixels=int(mask.sum()),finite_range_nonCAD_pixels=int(valid.sum()),
                bodyband_pixels=len(bw),target_cells=len(cc),contact_pixels=len(labels),visible_objects=len(object_visible),
                visible_grounded_no_qualified_contact_objects=len({k for k in object_visible if objects[k]['category']!='suspended'}-set(ids.tolist())),
                visible_suspended_objects=len({k for k in object_visible if objects[k]['category']=='suspended'}),**stats))
            save[eye+'_contact_uv']=labels;save[eye+'_CAD']=mask
        row=dict(frame=rr['frame'],time_s=float(a['source_time_s']),arms=arms,
            category_grid_instances={k:len(v) for k,v in categories.items()})
        for method in ['oracle','native']:
            m,dist=metric(sets[method],target,origin);row[method]=m;distances[method].extend(dist.tolist())
            save[method+'_union']=arr(sets[method])
        save.update(target_union=arr(target),origin=origin,root=a['root_position_physics_label'])
        for key,v in categories.items():save[key+'_cells']=arr(v)
        np.savez_compressed(out/rr['frame'],**save);rows.append(row)
        if len(rows)%10==1:print(condition,len(rows),'/',len(seal['rows']),'targets',len(target),'oracle',row['oracle']['recall'],'native',row['native']['recall'],flush=True)
    aggregates={};gates={}
    for method in ['oracle','native']:
        totals={key:sum(row[method][key] for row in rows) for key in ['prediction_cells','target_cells','correct','recovered']}
        precision=totals['correct']/totals['prediction_cells'] if totals['prediction_cells'] else None
        recall=totals['recovered']/totals['target_cells'] if totals['target_cells'] else None
        dist=np.array(distances[method]);p90=float(np.quantile(dist,.9)) if len(dist) and np.isfinite(dist).all() else None
        aggregates[method]=dict(**totals,precision=precision,recall=recall,position_p90_m=p90,
            contact_point_position_p90_m=float(np.quantile(point_errors[method],.9)) if point_errors[method] else None)
        gates[method]=dict(precision=precision is not None and precision>=.95,recall=recall is not None and recall>=.8,
            position=p90 is not None and p90<=.15,
            every_frame_recall=all(row[method]['target_cells']==0 or (row[method]['recall'] is not None and row[method]['recall']>=.8) for row in rows),
            every_frame_precision=all(row[method]['prediction_cells']==0 or row[method]['precision']>=.95 for row in rows))
    passed=all(all(g.values()) for g in gates.values())
    result=dict(complete=True,condition=condition,data_root=str(root),completion=completion,rows=rows,aggregate=aggregates,gates=gates,
        all_required_gates_pass=passed,decision='PASS_NECESSARY_DATA_AND_CONSUMER_GATE_ONLY' if passed else 'REJECT_BEFORE_FIT',
        object_categories={k:sum(ob['category']==k for ob in objects) for k in ['base_on_floor','floor_intersecting','suspended','below_floor']},
        visible_category_grid_instances={k:sum(row['category_grid_instances'][k] for row in rows) for k in categories},
        category_overlap='Categories can share a grid cell; union target denominator is counted separately. Unidentified surfaces never removed.',
        inherited_sensor_phase='Begin-labelled sensor packet is assigned interval end by existing estimator, retaining one-tick phase lag; no truth repair.',
        wall_s=time.monotonic()-start,fit=0,model=0,navigation=0,
        scope='Complete new acquired sequence, GT-only perfect contact-label source using rendered vs causal native camera pose. Not RGB performance, free space or navigation.',
        input_sha256={str(p):sha(p) for p in [root/'capture_seal.json',root/'native_imu.json',root/'scene_primitives.json',root/'physics.npz',D/'protocol.json']},
        source_sha256=sha(Path(__file__)))
    (D/(condition+'_qualification.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('FINAL',result['decision'],aggregates,gates,flush=True)

if __name__=='__main__':main()
