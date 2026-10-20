"""Offline independent labels; stereo predictor receives RGB and masks only."""
from pathlib import Path
import argparse
import json,time,hashlib
import numpy as np
from stereo_geometry import SGBMStereo,FOCAL,BASELINE
H=Path(__file__).resolve().parent
OLD=H.parent/'visual_information_new_rig_20261004_v2'

def summarize(rows):
    n=len(rows);valid=np.isfinite(rows)
    return dict(n=n,coverage=float(valid.mean()) if n else None,
                within_0p10m=float((rows<=.1).mean()) if n else None,
                within_0p30m=float((rows<=.3).mean()) if n else None,
                finite_p90_m=float(np.percentile(rows[valid],90)) if valid.any() else None,
                unconditional_p90_m=float(np.percentile(rows,90)) if n and valid.mean()>.9 else None)

def main():
    p=argparse.ArgumentParser();p.add_argument('--method',choices=['sgbm','fast'],default='sgbm');p.add_argument('--manifest',type=Path,default=OLD/'offline_manifest.json');p.add_argument('--tag',default='original');args=p.parse_args()
    manifest=json.loads(args.manifest.read_text())
    if args.method=='fast':
        from fast_stereo import FastStereo
        model=FastStereo()
    else:model=SGBMStereo()
    masks=np.load(OLD/'base_link_masks.npz')
    results=[];errors={};timings=[]
    y,x=np.indices((480,640));rays=np.stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(x.shape)),axis=-1)
    for number,row in enumerate(manifest['samples']):
        rgb=np.load(row['rgb_file']);truth=np.load(row['truth_file'])
        start=time.perf_counter();out=model.predict(rgb['left_rgb'],rgb['right_rgb'],masks['left'],masks['right']);timings.append(time.perf_counter()-start)
        z=truth['left_depth_m'];pose=truth['left_camera_usd_world']
        rotation=pose[:3,:3]@np.diag([1,-1,-1])
        world=pose[:3,3]+(rays*z[...,None])@rotation.T
        valid=np.isfinite(z)&(z>=.5)&(z<=3)&~masks['left']&(np.linalg.norm(world-truth['root_position_world_m'],axis=-1)>=.55)
        # Independent actual right-eye overlap/visibility label. Not passed to model.
        vr=y-FOCAL*BASELINE/np.maximum(z,1e-9)
        iv=np.rint(np.nan_to_num(vr,nan=-1,posinf=-1,neginf=-1)).astype(int)
        clipped=np.clip(iv,0,479)
        overlap=(vr>=0)&(vr<=479)&~masks['right'][clipped,x]
        rz=truth['right_depth_m'][clipped,x]
        overlap&=np.isfinite(rz)&(np.abs(rz-z)<=.03+.01*z)
        err=np.abs(out['depth_m']-z)*np.linalg.norm(rays,axis=-1)
        err=np.where(np.isfinite(err),err,np.inf)
        subgroups={'body_all':valid&(world[...,2]>=.05)&(world[...,2]<=1.5),
                   'body_overlap':valid&overlap&(world[...,2]>=.05)&(world[...,2]<=1.5),
                   'floor_all':valid&(np.abs(world[...,2])<=.05),
                   'floor_overlap':valid&overlap&(np.abs(world[...,2])<=.05)}
        frame={}
        for name,selection in subgroups.items():
            selection[1::2]=False;selection[:,1::2]=False
            values=err[selection];errors.setdefault(row['sequence'],{}).setdefault(name,[]).append(values)
            frame[name]=summarize(values)
        dest=H/('predictions' if args.tag=='original' else 'predictions_'+args.tag)/args.method/row['sequence'];dest.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(dest/Path(row['rgb_file']).name,**out)
        results.append(dict(sequence=row['sequence'],frame=Path(row['rgb_file']).name,latency_s=timings[-1],groups=frame))
        print('PAIR',number+1,len(manifest['samples']),row['sequence'],round(timings[-1],3),flush=True)
    groups={seq:{name:summarize(np.concatenate(values)) for name,values in sub.items()} for seq,sub in errors.items()}
    accepted=all(s['body_overlap']['n']>=100 and s['body_overlap']['within_0p10m']>=.9 for s in groups.values())
    result=dict(role='development audit; no fresh held-out or online admission',method=args.method,
                raw_epipolar='vertical',rectification='counterclockwise90deg; keep measured left/right order',baseline_m=BASELINE,focal_px=FOCAL,
                groups=groups,latency_p50_s=float(np.median(timings)),latency_p95_s=float(np.percentile(timings,95)),
                accuracy_gate_passed=accepted,online_admitted=False,frames=results,
                source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'stereo_geometry.py',args.manifest,OLD/'base_link_masks.npz']})
    result['warmup_s']=getattr(model,'warmup_s',None)
    (H/(args.method+'_audit'+('' if args.tag=='original' else '_'+args.tag)+'.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('STEREO_AUDIT_COMPLETE',groups,flush=True)
if __name__=='__main__':main()
