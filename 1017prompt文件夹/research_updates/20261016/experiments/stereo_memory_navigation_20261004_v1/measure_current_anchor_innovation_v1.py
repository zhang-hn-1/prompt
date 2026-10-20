"""Source-only old-geometry/current-appearance innovations; no correction or GT."""
from pathlib import Path
import datetime,hashlib,json
import numpy as np
import cv2
from strict_bilinear_support import unmasked_linear_support
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1'
S=H/'mvs_hero_temporal_development_v2_api_scale_recovery';D=H/'current_anchor_innovation_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not D.exists();D.mkdir()
paths=[S/'prediction_seal.json',Path(__file__),H/'strict_bilinear_support.py',
       H.parent.parent/'docs/superpowers/plans/2026-10-07-current-geometry-anchor-innovation.md']
for i in (235,245,255):
    paths += [O/k/f'frame_{i:03d}.npz' for k in ('appearance_predictions','known_poses','masks')]
    paths += [S/f'frame_{i:03d}_{eye}.npz' for eye in ('left','right')]
original=json.loads((S/'prediction_seal.json').read_text());assert all(sha(p)==v for p,v in original['output_sha256'].items())
(D/'input_custody.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256={str(p):sha(p) for p in paths},no_truth_input=True,development_not_fresh=True),indent=2)+'\n')
off=np.array([(x,y) for y in range(-3,4) for x in range(-3,4)])
rows=[]
for i in (235,245,255):
    f=np.load(O/'appearance_predictions'/f'frame_{i:03d}.npz');p=np.load(O/'known_poses'/f'frame_{i:03d}.npz');m=np.load(O/'masks'/f'frame_{i:03d}.npz')
    now=float(f['current_appearance_time_s']);old=float(f['source_geometry_time_s'])
    assert np.isfinite(now) and 0<=now-old<=10 and abs(float(p['time_s'])-now)<=1e-6 and abs(float(m['time_s'])-now)<=1e-6 and int(m['frame'])==i
    for ei,eye in enumerate(('left','right')):
        uv=f[eye+'_uv'];z=f[eye+'_z'];s=np.isfinite(f['patch_NCC'][:,ei])&(f['patch_NCC'][:,ei]>=.8)&(f['patch_std'][:,ei]>=5).all(1)&(z>0)
        projected=uv[:,None,:]+off
        support=unmasked_linear_support(m[eye],projected[:,:,0].astype(np.float32),projected[:,:,1].astype(np.float32)).all(1)
        s &= support;idx=np.flatnonzero(s);xy=uv[s]
        source=np.load(S/f'frame_{i:03d}_{eye}.npz');assert abs(float(source['current_time_s'])-now)<=1e-6
        depth=cv2.remap(source['depth_m'],xy[:,0,None].astype(np.float32),xy[:,1,None].astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=np.nan).ravel() if len(xy) else np.empty(0)
        innovation=depth-z[s];world=f['past_world'][s];bins=len(set(map(tuple,np.floor(world[:,:2]/.1).astype(int))))
        name=f'frame_{i:03d}_{eye}.npz'
        np.savez_compressed(D/name,old_world=world,current_uv=xy,old_expected_axial_m=z[s],Hero_sampled_axial_m=depth,
                            innovation_m=innovation,old_point_indices=idx,past_query_id=f['past_query_id'][s],
                            past_partner_id=f['past_partner_id'][s],old_geometry_time_s=old,current_time_s=now,
                            source_frame=f['source_frame'],new_geometry_or_correction=False)
        rows.append(dict(frame=i,eye=eye,file=name,n=len(idx),old_XY_bins=bins,time_s=now,geometry_time_s=old,
                         innovation_median_m=float(np.median(innovation)) if len(idx) else None,
                         innovation_p10_m=float(np.quantile(innovation,.1)) if len(idx) else None,
                         innovation_p90_m=float(np.quantile(innovation,.9)) if len(idx) else None,
                         no_calibration_or_new_point_prediction=True))
(D/'records.json').write_text(json.dumps(rows,indent=2)+'\n')
(D/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),output_sha256={str(p):sha(p) for p in D.iterdir() if p.is_file()},all_source_measurements_before_labels=True),indent=2)+'\n')
print(json.dumps(rows,indent=2))
