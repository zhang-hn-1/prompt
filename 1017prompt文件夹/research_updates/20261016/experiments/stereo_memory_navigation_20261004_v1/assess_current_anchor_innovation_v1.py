"""Measure whether source-selected old anchors are actually visible today, afterseal."""
from pathlib import Path
import datetime,hashlib,json
import numpy as np
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'current_anchor_innovation_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'prediction_seal.json').read_text());assert all(sha(p)==v for p,v in seal['output_sha256'].items())
rows=[]
for r in json.loads((D/'records.json').read_text()):
    f=np.load(D/r['file']);gt=np.load(O/'offline_truth'/f"frame_{r['frame']:03d}.npz");xy=np.rint(f['current_uv']).astype(int)
    T=gt[r['eye']+'_camera_usd_world'];z=gt[r['eye']+'_depth_m'][xy[:,1],xy[:,0]]
    ray=np.c_[(xy[:,0]-319.5)/584.8888888888889,(xy[:,1]-239.5)/584.8888888888889,np.ones(len(xy))]
    with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;error=np.linalg.norm(world-f['old_world'],axis=1)
    finite=np.isfinite(error);correct=finite&(error<=.1)
    rows.append(dict(**r,current_geometry_correct=int(correct.sum()),current_insufficient=int((~finite).sum()),
                     current_finite_wrong=int((finite&~correct).sum()),precision=float(correct.mean()) if len(xy) else None,
                     sufficient_local_anchor_support=bool(len(xy)>=20 and r['old_XY_bins']>=2),
                     finite_current_world_gap_median_m=float(np.median(error[finite])) if finite.any() else None))
valid=[r for r in rows if r['n']];passed=bool(valid) and all(r['precision']>=.95 and r['sufficient_local_anchor_support'] for r in valid)
target=D/'assessment_v1.json';assert not target.exists()
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,
            decision='LIMITED_CURRENT_ANCHOR_MEASUREMENT_ONLY' if passed else 'REJECT_CURRENT_ANCHOR_CALIBRATION_PROPOSAL',
            no_corrected_Hero_points_or_offset_model=True,no_left_eye_or_global_admission=True,
            old_bilateral_memory_and_Hero_REJECT_unchanged=True,author_postseal_diagnostic_only=True)
target.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
