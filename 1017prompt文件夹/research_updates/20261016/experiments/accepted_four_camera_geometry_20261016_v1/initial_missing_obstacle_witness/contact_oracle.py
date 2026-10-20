"""GT-only necessary geometry for an explicit visible obstacle-floor interface target.

Object geometry and renderer labels select ORACLE contact labels, never online input.
This does not rerun the rejected bottommost appearance-pixel detector.
"""
from pathlib import Path
import json,hashlib,time
import numpy as np
from scipy.spatial import cKDTree
from pxr import Usd,UsdGeom
from qualify import D,W,INPUT,NAMES,AX

sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
arr=lambda s:np.array(sorted(s),np.int64).reshape(-1,2)
st=lambda c:set(map(tuple,c.tolist()))

def main():
    start=time.monotonic();protocol=json.loads((D/'contact_target_protocol.json').read_text())
    cfg=protocol['new_target_oracle_only']
    result=json.loads((D/'geometry_result.json').read_text())
    seal=json.loads((INPUT/'recovery_run/capture_seal.json').read_text())
    F=seal['initial_local_identity'][0]['optics'][0]/18*640
    scene=Path('/home/an/project/NavDP/assets/scenes/benchmark_e1/e1_detour.usda')
    identity=json.loads((INPUT/'recovery_run/stage_identity.json').read_text())
    assert sha(scene)==next(v['sha256'] for v in identity['used_layers'] if v['real_path']==str(scene))
    stage=Usd.Stage.Open(str(scene));cache=UsdGeom.XformCache();objects=[]
    for p in stage.Traverse():
        if not p.IsA(UsdGeom.Cylinder):continue
        cylinder=UsdGeom.Cylinder(p);T=np.asarray(cache.GetLocalToWorldTransform(p)).T
        assert np.allclose(T[:3,:3],np.eye(3)) and cylinder.GetAxisAttr().Get()=='Z'
        objects.append(dict(path=str(p.GetPath()),center=T[:3,3].tolist(),radius=float(cylinder.GetRadiusAttr().Get()),height=float(cylinder.GetHeightAttr().Get())))
    theta=np.arange(1440)*2*np.pi/1440
    points=[];ids=[];grounded=[];suspended=[]
    for j,ob in enumerate(objects):
        c=np.array(ob['center']);bottom=c[2]-ob['height']/2
        if abs(bottom)>1e-6:suspended.append(j);continue
        grounded.append(j)
        points.append(np.c_[c[0]+ob['radius']*np.cos(theta),c[1]+ob['radius']*np.sin(theta),np.full(1440,bottom)])
        ids.extend([j]*1440)
    points=np.concatenate(points);ids=np.array(ids);rows=[];saved=[];errors=[]
    for row in result['frames']:
        a=np.load(INPUT/'recovery_run/frames'/row['frame']);audit=np.load(D/(Path(row['frame']).stem+'_audit.npz'))
        origin=audit['origin'];cell=lambda p:np.unique(np.floor((p[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
        arms=[];sets=[];save={};target=st(audit['all_union'])
        for name in NAMES:
            T=a[name+'_camera_usd_world_label'];R=T[:3,:3]@AX;z=a[name+'_depth_label'];mask=audit[name+'_CAD']
            c=(points-T[:3,3])@R;uv=c[:,:2]/np.maximum(c[:,2,None],1e-20)*F+[319.5,239.5]
            inside=(c[:,2]>=.5)&(c[:,2]<=3)&(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)
            ii=np.flatnonzero(inside);pix=np.rint(uv[ii]).astype(int);xx,yy=pix.T
            noncad=~mask[yy,xx];ii=ii[noncad];pix=pix[noncad];xx,yy=pix.T
            depth=z[yy,xx].astype(float);finite=np.isfinite(depth)&(depth>0)
            world=T[:3,3]+(np.c_[(pix-[319.5,239.5])/F,np.ones(len(pix))]*depth[:,None])@R.T
            contact_visible=finite&(np.linalg.norm(world-points[ii],axis=1)<=.02)
            # Direction of decreasing world height at the projected rim, including perspective.
            down=R.T@np.array([0.,0.,-1.])
            delta=down[:2]-c[ii,:2]/c[ii,2,None]*down[2]
            delta/=np.maximum(np.max(abs(delta),axis=1,keepdims=True),1e-20)
            floor_support=np.zeros(len(pix),bool)
            for step in [1,2,3]:
                neighbor=pix+np.rint(step*delta).astype(int);u,v=neighbor.T
                ok=(u>=0)&(u<640)&(v>=0)&(v<480);sel=np.flatnonzero(ok)
                if not len(sel):continue
                u=u[sel];v=v[sel];dd=z[v,u].astype(float)
                good=np.isfinite(dd)&(dd>0)&~mask[v,u]
                ww=T[:3,3]+(np.c_[(neighbor[sel]-[319.5,239.5])/F,np.ones(len(sel))]*dd[:,None])@R.T
                good&=(abs(ww[:,2])<=.01)
                # A floor witness must lie outside the physical obstacle footprint interior.
                for ob in objects:
                    cc=np.array(ob['center']);good&=np.linalg.norm(ww[:,:2]-cc[:2],axis=1)>=ob['radius']-.005
                floor_support[sel]|=good
            valid=contact_visible&floor_support
            labels=pix[valid];object_ids=ids[ii[valid]]
            # A perfect detector supplies only these pixel labels. Consumer does not use object shapes/depth.
            labels,index=np.unique(labels,axis=0,return_index=True);object_ids=object_ids[index]
            ray=np.c_[(labels-[319.5,239.5])/F,np.ones(len(labels))];direction=ray@R.T
            distance=np.divide(-T[2,3],direction[:,2],out=np.full(len(labels),np.inf),where=abs(direction[:,2])>1e-12)
            predicted=T[:3,3]+direction*distance[:,None]
            valid_ray=(distance>=.5)&(distance<=3)&np.isfinite(predicted).all(1)&(np.linalg.norm(predicted-a['root_position_physics_label'],axis=1)>=.55)
            labels=labels[valid_ray];object_ids=object_ids[valid_ray];predicted=predicted[valid_ray]
            actual_z=z[labels[:,1],labels[:,0]]
            actual=T[:3,3]+(np.c_[(labels-[319.5,239.5])/F,np.ones(len(labels))]*actual_z[:,None])@R.T
            metric=np.linalg.norm(predicted[:,:2]-actual[:,:2],axis=1);errors.extend(metric.tolist())
            cells=cell(predicted);sets.append(st(cells))
            visible_objects=set(object_ids.tolist())
            arms.append(dict(camera=name,projected_rim_samples=int(inside.sum()),nonCAD_rim_samples=len(ii),
                render_contact_supported_samples=int(contact_visible.sum()),
                floor_supported_rim_samples=int(valid.sum()),unique_contact_pixels=len(labels),
                visible_grounded_objects=len(visible_objects),grounded_objects_total=len(grounded),
                no_qualified_contact_objects=len(set(grounded)-visible_objects),suspended_objects_total=len(suspended),
                prediction_cells=len(cells),target_cells=len(audit[name+'_cells']),
                contact_position_error_p90_m=float(np.quantile(metric,.9)) if len(metric) else None))
            save[name+'_contact_uv']=labels;save[name+'_predicted_xy']=predicted[:,:2];save[name+'_cells']=cells
        union=set.union(*sets);pred=arr(union)*.1+origin;truth=arr(target)*.1+origin
        to_truth=cKDTree(truth).query(pred)[0] if len(pred) and len(truth) else np.full(len(pred),np.inf)
        to_pred=cKDTree(pred).query(truth)[0] if len(pred) and len(truth) else np.full(len(truth),np.inf)
        item=dict(frame=row['frame'],time_s=row['time_s'],arms=arms,prediction_cells=len(pred),target_cells=len(truth),
            prediction_correct=int((to_truth<=.15).sum()),target_recovered=int((to_pred<=.15).sum()),
            oracle_precision=float(np.mean(to_truth<=.15)) if len(pred) else None,
            oracle_recall=float(np.mean(to_pred<=.15)) if len(truth) else None,
            cell_position_p90_m=float(np.quantile(to_truth,.9)) if len(pred) else None)
        rows.append(item);save.update(oracle_union=arr(union),target_union=arr(target),origin=origin)
        np.savez_compressed(D/(Path(row['frame']).stem+'_contact_oracle.npz'),**save)
        print(row['frame'],len(pred),len(truth),item['oracle_recall'],flush=True)
    totals={k:sum(v[k] for v in rows) for k in ['prediction_cells','target_cells','prediction_correct','target_recovered']}
    precision=totals['prediction_correct']/totals['prediction_cells'] if totals['prediction_cells'] else None
    recall=totals['target_recovered']/totals['target_cells'] if totals['target_cells'] else None
    # P90 is over the union-cell domain, not contact pixels.
    distances=[]
    for row in rows:
        s=np.load(D/(Path(row['frame']).stem+'_contact_oracle.npz'));p=s['oracle_union']*.1+s['origin'];t=s['target_union']*.1+s['origin']
        if len(p):distances.extend(cKDTree(t).query(p)[0].tolist())
    p90=float(np.quantile(distances,.9)) if distances else None
    gates=dict(precision=precision is not None and precision>=.95,recall=recall is not None and recall>=.8,
        position=p90 is not None and p90<=.15,
        every_frame_recall=all(row['oracle_recall'] is not None and row['oracle_recall']>=.8 for row in rows))
    passed=all(gates.values())
    output=dict(complete=True,rows=rows,aggregate=totals,oracle_precision=precision,oracle_recall=recall,
        oracle_cell_position_p90_m=p90,contact_pixel_position_p90_m=float(np.quantile(errors,.9)) if errors else None,
        gates=gates,necessary_eight_pose_geometry_pass=passed,
        decision='PASS_EIGHT_POSE_NECESSARY_GEOMETRY_ONLY' if passed else 'REJECT_TARGET_BEFORE_FIT',
        training_admitted=False,online_source_admitted=False,full_sequence_coverage=False,
        missing=['Current four-camera full trajectories and independent held-out inputs/labels',
                 'Actual RGB visible-contact detector', 'Suspended/unknown-object and bottom-occlusion whole-domain qualification',
                 'Causal IMU/pose consumer qualification; current oracle uses renderer pose and true floor'],
        physical=0,fit=0,model=0,navigation=0,wall_s=time.monotonic()-start,
        objects=objects,grounded_object_count=len(grounded),suspended_object_count=len(suspended),
        unknown_geometry='Bodyband renderer pixels outside certified cylinder support remain unidentified; no full-scene semantic labels claimed',
        floor_geometry='z=0 is a disclosed prior, supported by saved floor pixels only; remote floor asset is not independently resolved',
        source_sha256=sha(Path(__file__)),protocol_sha256=sha(D/'contact_target_protocol.json'),scene_sha256=sha(scene))
    (D/'contact_oracle_result.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    print({k:v for k,v in output.items() if k in ['decision','oracle_precision','oracle_recall','oracle_cell_position_p90_m','gates','wall_s']},flush=True)

if __name__=='__main__':main()
