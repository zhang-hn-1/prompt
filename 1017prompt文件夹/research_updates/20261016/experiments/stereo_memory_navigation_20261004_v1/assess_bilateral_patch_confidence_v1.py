from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;D=H/'controlled_heading_probe_run_v1';O=H/'bilateral_patch_confidence_v1';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();seal=json.loads((O/'prediction_seal_v1.json').read_text());assert all(sha(f)==s for f,s in seal['input_sha256'].items());assert all(sha(f)==s for f,s in seal['output_sha256'].items())
def geometry(d,p):
 w=p['world'];haz=(w[:,2]>.05)&(w[:,2]<=1.5);w=w[haz];i=int(p['frame']);gt=np.load(d/'offline_truth'/f'frame_{i:03d}.npz');es=[]
 for eye,key in [('left','query_pixel_id'),('right','partner_pixel_id')]:
  idx=p[key][haz];x,y=idx%640,idx//640;z=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world'];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;es.append(np.linalg.norm(actual-w,axis=1))
 e=np.asarray(es).T;finite=np.isfinite(e).all(1);ok=finite&(e<=.1).all(1)
 return dict(n=len(w),correct=int(ok.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~ok).sum()),precision=float(ok.mean()) if len(w) else None,p90_finite_max_endpoint_error_m=float(np.quantile(e[finite].max(1),.9)) if finite.any() else None,correct_XY_bins=len(np.unique(np.floor(w[ok,:2]/.1).astype(int),axis=0)))


rows=[]
for f in sorted(O.glob('source_*.npz')):
 p=np.load(f);g=geometry(D,p);rows.append(dict(frame=int(p['frame']),time_s=float(p['time_s']),score=g))
nonempty=[r['score'] for r in rows if r['score']['n']];precisionpass=bool(nonempty) and all(s['precision']>=.95 for s in nonempty);pairs=[(a['frame'],b['frame']) for a,b in zip(rows[:-1],rows[1:]) if b['frame']-a['frame']==5 and all(r['score']['n']>=100 and r['score']['precision']>=.95 and r['score']['correct_XY_bins']>=2 for r in [a,b])];passed=precisionpass and bool(pairs)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_HISTORICAL_PREFLIGHT_NOT_FRESH' if passed else 'REJECT_FIXED_PATCH_CONFIDENCE',precision_all_nonempty_ge95=precisionpass,two_consecutive_source_coverage_pass=bool(pairs),qualifying_pairs=pairs,rows=rows,diagnostics=seal['diagnostics'],extra_NN_calls=0,extra_physics_sequences=0,historical_development_not_fresh=True,old_source_gate_unchanged=True,no_threshold_rescue_or_other_confidence_candidate=True,formal_triggered=False,whole_research_complete=False,script_sha256=sha(__file__),prediction_seal_sha256=sha(O/'prediction_seal_v1.json'))
(O/'assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(decision=result['decision'],rows=rows,precisionpass=precisionpass,pairs=pairs),indent=2))
