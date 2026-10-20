"""Post-seal development labels; insufficient endpoints stay in denominator."""
from pathlib import Path
import hashlib,json,datetime,numpy as np
H=Path(__file__).resolve().parent;run=H/'observation_handoff_observe_run_recovery_v2';out=H/'past_geometry_anchor_predictions_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seal=json.loads((out/'all_predictions_seal.json').read_text())
assert all(sha(p)==s for p,s in seal['prediction_SHA256'].items());rows=[]
for p in sorted(out.glob('frame_*.npz')):
 pred=np.load(p);i=int(pred['frame'])
 if i<180:continue
 src=np.load(run/'sensor_packets'/f'source_{i:03d}.npz');truth=np.load(run/'offline_truth'/f'frame_{i:03d}.npz');q=src['query_pixel_id'].astype(int);yy=q//640;xx=q%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]
 with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(src['world']-actual,axis=1)
 hazard=(src['world'][:,2]>.05)&(src['world'][:,2]<=1.5);keep=pred['keep'];good=np.isfinite(err)&(err<=.1);assert not (keep&~hazard).any()
 rows.append(dict(frame=i,baseline_hazards=int(hazard.sum()),baseline_correct=int((hazard&good).sum()),retained=int(keep.sum()),retained_correct=int((keep&good).sum()),retained_insufficient=int((keep&~np.isfinite(err)).sum()),retained_proven_error=int((keep&np.isfinite(err)&~good).sum())))
def metrics(rows):
 sums={k:sum(r[k] for r in rows) for k in rows[0] if k!='frame'};sums['precision']=sums['retained_correct']/sums['retained'] if sums['retained'] else None;sums['correct_retention']=sums['retained_correct']/sums['baseline_correct'] if sums['baseline_correct'] else None;return sums
m=metrics(rows);passed=m['precision'] is not None and m['correct_retention'] is not None and m['precision']>=.95 and m['correct_retention']>=.80
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),prediction_seal_SHA256=sha(out/'all_predictions_seal.json'),gate=dict(precision=.95,correct_retention=.80),metrics=m,without_seed=metrics(rows[1:]),rows=rows,decision='DEVELOPMENT_PASS_ONLY' if passed else 'REJECT',not_fresh=True,no_navigation_result=True,insufficient_is_not_proven_error=True)
(H/'past_geometry_anchor_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
