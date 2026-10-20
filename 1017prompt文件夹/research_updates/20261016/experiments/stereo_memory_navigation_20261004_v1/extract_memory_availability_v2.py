"""Fixed24-triplet retrospective availability dataset, no fit or actor.

Feature artifact is sealed BEFORE any future teacher captures are opened.
Teacher absence never certifies a free memory cell.
"""
from pathlib import Path
import argparse,collections,datetime,hashlib,json,sys
import numpy as np
from memory_availability_features_v1 import past_only_features
from memory_temporal_ray_labels_v2 import label_ray
from memory_availability_identity_v1 import verify_identity
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def sealed_load(D,relative,manifest,identities):
    p=D/relative
    digest=sha(p)
    if manifest[str(p)]!=digest:raise ValueError('original input seal mismatch: '+str(p))
    identities[str(p)]=digest
    return dict(np.load(p,allow_pickle=False))

def feature_stage():
    verify_identity(H/'memory_availability_extraction_source_identity_v2.json')
    design=json.loads((H/'memory_availability_forecast_design_v1.json').read_text())
    identities={};rows=[];features=[];points=[];triplets=[]
    for split,names in design['sequences'].items():
        for name in names:
            D=H/name;m=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
            for tri in design['triplets']:
                p,c,f=(tri[k] for k in ('past_frame','current_frame','teacher_frame'))
                past=sealed_load(D,f'partial_observations/frame_{p:03d}.npz',m,identities)
                current=sealed_load(D,f'partial_observations/frame_{c:03d}.npz',m,identities)
                pose=sealed_load(D,f'known_poses/frame_{c:03d}.npz',m,identities)
                history=sealed_load(D,f'history/frame_{c:03d}.npz',m,identities)
                grid=sealed_load(D,f'maps/frame_{c:03d}.npz',m,identities)
                t=float(grid['time_s'])
                if abs(t-c*.1)>1e-7 or abs(float(past['acquisition_time_s'])-p*.1)>1e-7:raise ValueError('fixed frame clock mismatch')
                # History is captured synchronously at this current source frame;
                # no current/future request is loaded in this stage.
                r=past_only_features(past=past,current=current,pose=pose,history=history,history_time_s=t,current_time_s=t,memory_origin=grid['origin'],memory_resolution=float(grid['resolution']))
                begin=len(rows)
                for i in r['source_indices']:
                    rows.append(dict(sequence=name,split=split,past_frame=p,current_frame=c,teacher_frame=f,current_time_s=t,past_time_s=float(past['acquisition_time_s']),source_index=int(i),past_query_pixel_id=int(past['query_pixel_id'][i]),memory_origin=grid['origin'].tolist(),memory_resolution=float(grid['resolution'])))
                features.append(r['features']);points.append(r['points_world'])
                triplets.append(dict(sequence=name,split=split,**tri,past_points=min(64,len(past['world'])),selected=len(r['source_indices']),row_start=begin,row_end=len(rows)))
    out=H/'memory_availability_past_only_features_v1.npz';index=H/'memory_availability_past_only_index_v1.json'
    if out.exists() or index.exists():raise FileExistsError('fixed feature extraction budget already exists')
    np.savez_compressed(out,features=np.concatenate(features),points_world=np.concatenate(points))
    index.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,triplets=triplets,feature_inputs_sha256=identities,feature_artifact_sha256=sha(out),design_sha256=sha(H/'memory_availability_forecast_design_v1.json'),new_fit=0,new_physics=0,future_teacher_opened=False),indent=2)+'\n')
    receipt=H/'memory_availability_feature_receipt_v1.json'
    if receipt.exists():raise FileExistsError('feature receipt already exists')
    receipt.write_text(json.dumps({str(p):sha(p) for p in [out,index,H/'memory_availability_forecast_design_v1.json',H/'memory_availability_extraction_source_identity_v2.json']},indent=2)+'\n')
    print(dict(feature_rows=len(rows),triplets=len(triplets),feature_artifact=str(out)))

def teacher_stage():
    verify_identity(H/'memory_availability_extraction_source_identity_v2.json')
    verify_identity(H/'memory_availability_feature_receipt_v1.json')
    output=H/'memory_availability_teacher_dataset_v1.json'
    if output.exists():raise FileExistsError('fixed teacher extraction budget already exists')
    indexfile=H/'memory_availability_past_only_index_v1.json';index=json.loads(indexfile.read_text());features=H/'memory_availability_past_only_features_v1.npz'
    if index['feature_artifact_sha256']!=sha(features):raise ValueError('features changed before teacher stage')
    points=np.load(features,allow_pickle=False)['points_world']
    DEP=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310');sys.path.insert(0,str(DEP))
    from timed_static_base_self_mask import TimedStaticBaseSelfMask
    from dynamic_self_mask import joints_from_proprio
    cad=TimedStaticBaseSelfMask();identities={};labels=[];diagnostics=[];counts={}
    for tri in index['triplets']:
        name=tri['sequence'];D=H/name;m=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];f=tri['teacher_frame'];c=tri['current_frame']
        # Retrospective contact qualification belongs only to the teacher stage.
        contacts=json.loads((D/'physics_contacts.json').read_text())
        if not contacts or not all(row['available'] for row in contacts):raise ValueError('contact truth unavailable')
        collision=min([row['simulation_time_s'] for row in contacts if row['contact']]+[35.])
        if f*.1>=collision:raise ValueError('fixed teacher would cross actual first collision')
        capture=sealed_load(D,f'frames/frame_{f:03d}.npz',m,identities)
        history=sealed_load(D,f'history/frame_{f:03d}.npz',m,identities)
        teacher_t=float(capture['time_s'])
        if abs(teacher_t-f*.1)>1e-7 or teacher_t<=c*.1:raise ValueError('invalid later teacher clock')
        q=joints_from_proprio(history['proprio_history']);mask=cad.project(q)['left']
        reasons=collections.Counter();local=[]
        for i in range(tri['row_start'],tri['row_end']):
            row=index['rows'][i]
            r=label_ray(point_world=points[i],camera_usd_world=capture['left_camera_usd_world'],depth_m=capture['left_depth_m'],self_mask=mask,past_time_s=row['current_time_s'],current_time_s=teacher_t,memory_origin=row['memory_origin'],memory_resolution=row['memory_resolution'])
            y=1 if r['reason']=='visible_same_cell' else -1 if r['reason']=='invalid_depth' else 0
            labels.append(y);local.append(y);reasons[r['reason']]+=1
            diagnostics.append(dict(row=i,availability_label=y,**r))
        counts.setdefault(name,collections.Counter()).update(local)
        tri['teacher_reason_counts']=dict(reasons);tri['teacher_mask_sha256']=hashlib.sha256(mask.tobytes()).hexdigest();tri['teacher_joints']=q.tolist()
    covered={name:dict(visible=c[1],unavailable=c[0],missing_teacher=c[-1]) for name,c in counts.items()}
    development=[r['availability_label'] for r,row in zip(diagnostics,index['rows']) if row['split']=='fit_development']
    held=[name for name in counts if name.startswith('native_pose') and '_e0_' not in name]
    gate=len(held)==2 and all(counts[name][1]>=20 and counts[name][0]>=20 for name in held) and development.count(1)>=40 and development.count(0)>=40
    output.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),classification='availability_dataset_coverage_pass_no_fit' if gate else 'unassessed_availability_label_coverage',coverage=covered,development=dict(visible=development.count(1),unavailable=development.count(0),missing_teacher=development.count(-1)),rows=len(labels),labels=labels,diagnostics=diagnostics,triplets=index['triplets'],teacher_inputs_sha256=identities,feature_index_sha256=sha(indexfile),feature_artifact_sha256=sha(features),no_future_teacher_in_features=True,new_fit=0,new_physics=0,unknown_unavailable_never_free=True,whole_mission_complete=False),indent=2)+'\n')
    print(dict(classification='coverage_pass_no_fit' if gate else 'unassessed_coverage',coverage=covered,rows=len(labels)))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['features','teacher']);args=parser.parse_args()
    (feature_stage if args.stage=='features' else teacher_stage)()
