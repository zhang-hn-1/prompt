"""One bounded archived-query measurement; truth labels never enter a predictor/map."""
from pathlib import Path
import datetime,hashlib,json
import numpy as np
from scene_ray_geometry import camera_rays
from past_multiview import measure_view,measure_history

H=Path(__file__).resolve().parent
STATUS=['behind_or_invalid_query','outside_full_footprint','body_mask_footprint','missing_depth_footprint','inconsistent_depth_footprint','visible']

def aggregate(values):
    n=len(values['current_stereo']);result=dict(n=n)
    for k in ['current_left','current_right','current_stereo','current_rgb','past_any_eye','past_stereo','past_rgb_reliable']:
        result[k+'_n']=int(values[k].sum());result[k+'_fraction']=float(values[k].mean()) if n else None
    for name,current,past in [('single_eye','current_left','past_any_eye'),('stereo','current_stereo','past_stereo'),('rgb','current_rgb','past_rgb_reliable')]:
        gained=~values[current]&values[past]
        result[name+'_past_gain_n']=int(gained.sum())
        result[name+'_union_n']=int((values[current]|values[past]).sum())
        result[name+'_past_gain_fraction_all']=float(gained.mean()) if n else None
    result['current_right_partitions']={}
    for code,name in enumerate(STATUS):
        selected=values['current_right_status']==code;count=int(selected.sum())
        result['current_right_partitions'][name]=dict(n=count,
            past_any_eye_n=int((selected&values['past_any_eye']).sum()),
            past_stereo_n=int((selected&values['past_stereo']).sum()),
            past_rgb_reliable_n=int((selected&values['past_rgb_reliable']).sum()))
    unavailable=~values['current_stereo'];denom=int(unavailable.sum())
    result['current_stereo_unavailable']=dict(n=denom,
        earlier_any_eye_n=int((unavailable&values['past_any_eye']).sum()),
        earlier_stereo_n=int((unavailable&values['past_stereo']).sum()),
        earlier_rgb_reliable_n=int((unavailable&values['past_rgb_reliable']).sum()))
    return result


def main():
    protocol=json.loads((H/'past_multiview_protocol.json').read_text())
    def check():
        for path,sha in protocol['input_sha256'].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,path
    check();start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    manifest=json.loads((H/'transaction_development_manifest.json').read_text())
    samples=sorted([r for r in manifest['samples'] if r['sequence']==protocol['sequence']],key=lambda r:Path(r['rgb_file']).name)
    assert [int(Path(s['rgb_file']).stem.split('_')[1]) for s in samples]==protocol['frames']
    source=np.load(H/'static_scene_ray_measurements.npz');mesh=np.load(H/'static_scene_mesh_measurements.npz')
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    masks={eye:masks[eye] for eye in ['left','right']}
    past=[];arrays={};rows=[];pooled={};querytotal=0
    for row in samples:
        # Load CURRENT pair here, never load later image/truth/prediction files into history.
        frame=int(Path(row['rgb_file']).stem.split('_')[1]);truth=np.load(row['truth_file']);rgb=np.load(row['rgb_file'])
        pred=np.load(H/'predictions_transaction_development/fast'/protocol['sequence']/Path(row['rgb_file']).name)
        t=float(truth['simulation_time_s']);assert t==float(rgb['simulation_time_s'])
        assert rgb['left_rgb'].shape==rgb['right_rgb'].shape==(480,640,3)
        if past:assert t>past[-1]['time_s']
        pair=dict(time_s=t,frame=frame,left_pose=truth['left_camera_usd_world'],right_pose=truth['right_camera_usd_world'],
            left_depth=truth['left_depth_m'],right_depth=truth['right_depth_m'],predicted_left=pred['depth_m'],confidence_left=pred['confidence'])
        prefix=f'left_{frame:03d}';selection=source[prefix+'_sphere_core_selection']
        origin,rays,yy,xx=camera_rays(pair['left_pose'],480,640,protocol['focal_px'],*protocol['principal_xy'],protocol['pixel_stride'])
        depth=mesh[prefix+'_raw_mesh_axis_depth_m'][selection]
        points=origin+depth[:,None]*rays[selection]
        assert np.isfinite(points).all();querytotal+=len(points)
        params=(protocol['focal_px'],*protocol['principal_xy'],protocol['visibility_tolerance_ray_m'])
        left=measure_view(points,pair['left_pose'],pair['left_depth'],masks['left'],*params,pair['predicted_left'],pair['confidence_left'],protocol['rgb_label_tolerance_ray_m'])
        right=measure_view(points,pair['right_pose'],pair['right_depth'],masks['right'],*params)
        history=measure_history(points,t,past,masks,*params,protocol['rgb_label_tolerance_ray_m'])
        # Actual query points + actual archived past bundles, poisoned future payloads.
        # Current/future metadata is inspected only to exclude them; their fields are not read.
        poison=[dict(time_s=t,frame=frame,left_pose='poison current'),dict(time_s=t+1,frame=999,left_pose='poison future')]
        replaced=measure_history(points,t,past+poison,masks,*params,protocol['rgb_label_tolerance_ray_m'])
        for key,value in history.items():np.testing.assert_equal(value,replaced[key])
        data=dict(current_left=left['visible'],current_right=right['visible'],current_stereo=left['visible']&right['visible'],
            current_rgb=left['rgb_accurate']&right['visible'],current_left_status=left['status'],current_right_status=right['status'],
            current_rgb_supported=left['rgb_supported'],current_rgb_error_m=left['rgb_error_m'],
            **{'past_'+key:val for key,val in history.items()})
        for key,val in data.items():arrays[f'frame_{frame:03d}__'+key]=val;pooled.setdefault(key,[]).append(val)
        arrays[f'frame_{frame:03d}__source_world_m']=points
        arrays[f'frame_{frame:03d}__source_pixels_yx']=np.column_stack([yy[selection],xx[selection]])
        arrays[f'frame_{frame:03d}__source_scene_id']=source[prefix+'_expected_scene_id'][selection]
        for kind in ['any','stereo','rgb']:
            tt=history['first_'+kind+'_time_s'];ff=history['first_'+kind+'_frame'];valid=ff>=0
            assert (tt[valid]<t).all() and set(ff[valid]).issubset({p['frame'] for p in past})
        summary=aggregate(data)
        rows.append(dict(frame=frame,time_s=t,loaded_history_frames=[p['frame'] for p in past],loaded_history_times_s=[p['time_s'] for p in past],
            source_query_eye='left',source_point_identity='same frozen raw mesh intersection/old full ideal core',
            future_append_poison_exact=True,metrics=summary))
        past.append(pair)
        print('PAST_QUERY',frame,'time',t,'history',[p['frame'] for p in past[:-1]],summary,flush=True)
    pooled={k:np.concatenate(v) for k,v in pooled.items()};assert querytotal==73383
    result=dict(role=protocol['role'],started_utc=start,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,
        pooled=aggregate(pooled),all_history_strict_past=True,future_append_poison_exact=True,
        new_simulations=0,new_inference_calls=0,online_admitted=False,old_gates_changed=False,
        limitations=protocol['limitations'],source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'past_multiview.py',H/'past_multiview_protocol.json']})
    np.savez_compressed(H/'past_multiview_measurements.npz',**arrays)
    (H/'past_multiview_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    check();print('COMPLETE',result['ended_utc'],'queries',querytotal,'no future data / no online admission')

if __name__=='__main__':main()
