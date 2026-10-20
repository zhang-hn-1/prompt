from pathlib import Path
import sys,json,hashlib,time
D=Path(__file__).resolve().parent;W=D.parents[1];S=W/'experiments/four_camera_weak_baseline_20261017_v1';sys.path[:0]=[str(S),str(D)]
from paths import *
# paths exports its own D; restore this experiment root explicitly.
D=W/'experiments/view_supported_memory_20261017_v1'
from source import NAMES,LOCAL
from memory import ViewSupportMemoryGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from current_exposure_risk_grid_v1 import CurrentExposureRiskGrid
import numpy as np
from scipy.spatial import cKDTree
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def checks():
    def r(f,t,x):return dict(frame=f,time_s=t,query_camera='front_left',world=np.array([[1.,0.,.4]]),body_center=np.array([0.,x,.4]),pair_center=np.array([.18,x,.515]))
    def feed(g,rows,f,t):g.set_source(dict(frame=f,time_s=t,records=rows));g.update_points(np.empty((0,3)),np.array([0.,0.,.4]),t)
    g=ViewSupportMemoryGrid([-20,-20],10,static_scene_assumption=True)
    feed(g,[r(0,0,0)],0,0);feed(g,[r(0,0,0),r(1,.5,0)],1,.5);assert not np.isfinite(g._view_clock).any()
    feed(g,[],2,11.);assert not (g.labels==0).any() and not (g.labels==1).any()
    g=ViewSupportMemoryGrid([-20,-20],10,static_scene_assumption=True)
    feed(g,[r(0,0,0),r(1,.5,.21)],1,.5);assert np.isfinite(g._view_clock).sum()==1
    feed(g,[],2,11.);assert (g.labels==1).sum()==1 and not (g.labels==0).any()
    before=g.labels.copy()
    try:feed(g,[r(3,12,0)],2,11.5)
    except ValueError:pass
    else:raise AssertionError('future accepted')
    assert np.array_equal(g.labels,before)
    for dx in [-.05,.05]:
        h=ViewSupportMemoryGrid([-20,-20],10,static_scene_assumption=True);feed(h,[r(0,0,0),r(1,.5,dx)],1,.5);assert not np.isfinite(h._view_clock).any()
    return dict(stationary_repetition_not_confirmation=True,duplicate_exposure_not_confirmation=True,single_view_expiry_unknown_not_free=True,translation_and_angle_confirm=True,blind_confirmed_retained=True,future_before_payload_rejected=True,fixture_5cm_pose_perturbation_not_confirmation=True,fixture_not_pose_error_bound=True)
def main():
    cfg=json.loads((D/'protocol.json').read_text())
    for p,h in cfg['source_sha256'].items():assert sha(p)==h,p
    seal=json.loads((S/'prediction_seal.json').read_text());assert seal['complete'];out=D/'maps';out.mkdir(exist_ok=False)
    check=checks();first=np.load(S/'e0_predictions/frame_000.npz');origin=first['body_estimated'][:2,3]-20
    grids={k:cls(origin,10,static_scene_assumption=True) for k,cls in [('A',CurrentExposureRiskGrid),('B',RepeatedExposureMemoryGrid),('C',ViewSupportMemoryGrid)]}
    history=[];rows=[];source_hashes={};start=time.monotonic()
    for i,rr in enumerate(seal['rows']):
        f=S/'e0_predictions'/rr['frame'];assert sha(f)==rr['prediction_sha256'];source_hashes[str(f)]=sha(f)
        with np.load(f) as p:
            body=p['body_estimated'];t=float(p['time_s']);new=[]
            for j,n in enumerate(NAMES):
                pair_center_body=(LOCAL[0][:3,3]+LOCAL[1][:3,3])/2 if n.startswith('front') else (LOCAL[2][:3,3]+LOCAL[3][:3,3])/2
                new.append(dict(frame=i,time_s=t,query_camera=n,world=p[n+'_world'],query_pixel_id=p[n+'_query_pixel'],partner_pixel_id=p[n+'_partner_pixel'],body_center=body[:3,3],pair_center=body[:3,3]+body[:3,:3]@pair_center_body))
            history=[r for r in history if t-r['time_s']<=10]+new
            row=dict(frame=i,time_s=t);save={}
            for k,g in grids.items():
                g.set_source(dict(frame=i,time_s=t,records=history));g.update_points(np.empty((0,3)),body[:3,3],t)
                row[k]=g.diagnostics();save[k+'_labels']=g.labels.copy();save[k+'_codes']=g.codes.copy()
            save.update(origin=origin,resolution=.1,time_s=t);dest=out/rr['frame'];np.savez_compressed(dest,**save);row['sha256']=sha(dest);rows.append(row)
    # Seal all memory outputs before loading any GT/occupancy for performance scoring.
    (D/'replay_seal.json').write_text(json.dumps(dict(complete=True,rows=rows,checks=check,source_prediction_sha256=source_hashes,source_sha256=cfg['source_sha256'],wall_s=time.monotonic()-start,GT_decision=False),indent=2)+'\n')
    print('REPLAY_SEALED',len(rows),check,flush=True)
if __name__=='__main__':main()
