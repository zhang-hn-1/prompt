"""After-the-fact source-query labels; never imported by RGB predictor/runner."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from past_multiview import measure_view
H=Path(__file__).resolve().parent

def summary(data):
    n=len(data['error_m']);out=dict(n=n)
    for key in ['confident','correct','strict_any_visible_contributor','strict_all_visible_contributors','fast_confident','fast_correct']:
        out[key+'_n']=int(data[key].sum());out[key+'_fraction_all']=float(data[key].mean())
    for name,key,good in [('candidate','error_m','confident'),('raw','raw_error_m','raw_finite'),('fast','fast_error_m','fast_confident')]:
        e=data[key][data[good]&np.isfinite(data[key])]
        out[name+'_finite_n']=int(len(e));out[name+'_ray_error_median_m']=float(np.median(e)) if len(e) else None
        out[name+'_ray_error_p90_m']=float(np.percentile(e,90)) if len(e) else None
        out[name+'_conditional_within_0p10m']=float(np.mean(e<=.1)) if len(e) else None
    for key in ['no_raw','boundary_best','cost_rejected','interval_rejected','source_true_depth_out_of_grid']:
        out[key+'_n']=int(data[key].sum())
    out['gate_pass']=bool(n>=100 and out['correct_fraction_all']>=.9 and out['candidate_ray_error_p90_m'] is not None and out['candidate_ray_error_p90_m']<=.1)
    return out

def main():
    p=json.loads((H/'multiview_rgb_protocol.json').read_text());predmanifest=json.loads((H/'multiview_rgb_predictions/manifest.json').read_text())
    assert predmanifest['all_four_predictions_serialized_before_truth_evaluation']
    started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    assert all(r['ended_utc']<started for r in predmanifest['rows'])
    for file,sha in p['input_sha256'].items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==sha,file
    old=np.load(H/'past_multiview_measurements.npz');masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    truths={};rgbmanifest=json.loads((H/'transaction_development_manifest.json').read_text())
    for r in rgbmanifest['samples']:
        if r['sequence']==p['sequence']:truths[int(Path(r['rgb_file']).stem.split('_')[1])]=np.load(r['truth_file'])
    arrays={};pooled={};rows=[]
    for r in predmanifest['rows']:
        f=r['frame'];pre=f'frame_{f:03d}__';points=old[pre+'source_world_m'];yx=old[pre+'source_pixels_yx'];yy,xx=yx.T
        pred=np.load(r['prediction']);assert hashlib.sha256(Path(r['prediction']).read_bytes()).hexdigest()==r['sha256']
        truth=truths[f];vis=measure_view(points,truth['left_camera_usd_world'],truth['left_depth_m'],masks['left'],
            p['focal_px'],p['cx'],p['cy'],.01,pred['depth_m'],pred['confidence'],.1)
        z=vis['axis_depth_m'];norm=np.sqrt(1+((xx-p['cx'])/p['focal_px'])**2+((yy-p['cy'])/p['focal_px'])**2)
        def centre(depth,confidence):
            values=depth[yy,xx];conf=confidence[yy,xx]&np.isfinite(values)&(values>0)
            err=abs(values-z)*norm;err[~conf]=np.inf
            return conf,err
        conf,error=centre(pred['depth_m'],pred['confidence']);rawfinite,rawerror=centre(pred['raw_depth_m'],np.ones((480,640),bool))
        fast=np.load(H/'predictions_transaction_development/fast'/p['sequence']/f'frame_{f:03d}.npz');fc,fe=centre(fast['depth_m'],fast['confidence'])
        # Every prediction sample's used views are accounted for; labels remain offline only.
        visiblebits=np.zeros(len(points),np.uint8)
        for source in r['sources']:
            t=truths[source['frame']];eye=source['eye']
            labels=measure_view(points,t[eye+'_camera_usd_world'],t[eye+'_depth_m'],masks[eye],p['focal_px'],p['cx'],p['cy'],.01)
            visiblebits|=labels['visible'].astype(np.uint8)*np.uint8(source['bit'])
        x0=np.floor(np.clip(vis['xy'][:,0],0,638)).astype(int);y0=np.floor(np.clip(vis['xy'][:,1],0,478)).astype(int)
        bb=np.stack([pred['view_bits'][y0,x0],pred['view_bits'][y0+1,x0],pred['view_bits'][y0,x0+1],pred['view_bits'][y0+1,x0+1]])
        common=np.bitwise_and.reduce(bb,axis=0);used=np.bitwise_or.reduce(bb,axis=0)
        anycontributor=(common&visiblebits)!=0
        allcontributors=(used!=0)&((used&visiblebits)==used)
        raw=pred['raw_depth_m'][yy,xx];cost=pred['cost'][yy,xx];interval=pred['interval_ray_width_m'][yy,xx]
        d=dict(confident=conf,error_m=error,correct=conf&(error<=.1),raw_finite=rawfinite,raw_error_m=rawerror,
            strict_any_visible_contributor=vis['rgb_accurate']&anycontributor,
            strict_all_visible_contributors=vis['rgb_accurate']&anycontributor&allcontributors,
            fast_confident=fc,fast_error_m=fe,fast_correct=fc&(fe<=.1),visible_source_bits=visiblebits,
            common_used_bits=common,all_used_bits=used,no_raw=~rawfinite,
            boundary_best=rawfinite&((raw<=p['depth_min_m']+1e-7)|(raw>=p['depth_max_m']-1e-7)),
            cost_rejected=rawfinite&(cost>p['max_cost']),interval_rejected=rawfinite&(interval>p['max_interval_ray_width_m']),
            source_true_depth_out_of_grid=(z<p['depth_min_m'])|(z>p['depth_max_m']))
        metrics=summary(d);rows.append(dict(frame=f,time_s=r['time_s'],sources=r['sources'],elapsed_cpu_wall_s=r['elapsed_cpu_wall_s'],
            confident_image_pixels=r['confident_pixels'],metrics=metrics))
        for k,v in d.items():arrays[pre+k]=v;pooled.setdefault(k,[]).append(v)
        arrays[pre+'source_pixels_yx']=yx
        print('EVALUATION',f,metrics,flush=True)
    pooled={k:np.concatenate(v) for k,v in pooled.items()};metrics=summary(pooled);assert metrics['n']==73383
    accepted=all(r['metrics']['gate_pass'] for r in rows)
    result=dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),role=p['role'],rows=rows,pooled=metrics,
        candidate_accepted=accepted,online_navigation_admitted=False,old_gates_changed=False,new_simulations=0,new_control_runs=0,
        fresh_sequences=0,known_pose_only=True,prediction_truth_separation='all four serialized before assessment; runner reads sanitized packets only',
        strict_provenance='any: one common used view genuinely visible; all: every used view across entire 2x2 output footprint visible; both also require whole current-left footprint correct <=.10m and GT visible <=.01m',
        rejection_counters='overlapping, NOT disjoint partitions; no_raw excludes later cost/interval counters',
        source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),H/'multiview_rgb.py',H/'multiview_rgb_protocol.json',H/'multiview_rgb_predictions/manifest.json']})
    np.savez_compressed(H/'multiview_rgb_measurements.npz',**arrays)
    (H/'multiview_rgb_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('CANDIDATE', 'ACCEPT' if accepted else 'REJECT', metrics,flush=True)
if __name__=='__main__':main()
