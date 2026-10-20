from pathlib import Path
import json,hashlib,time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
D=Path(__file__).resolve().parent;W=D.parents[1]
H=W/'experiments/contact_boundary_source_20261016_v1'
R=H/'e0_recovery_run'
EYES=['front_left','front_right','rear_left','rear_right']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
start=time.monotonic();result=json.loads((D/'result.json').read_text());seal=json.loads((D/'prediction_seal.json').read_text());ins=json.loads((D/'ideal_inputs_seal.json').read_text())
assert len(result['rows'])==len(seal['rows'])==len(ins['rows'])==110
counts={a:dict(predicted=0,target=0,correct=0,covered=0) for a in ['render','native']}
err={a:[] for a in counts};dist={a:[] for a in counts};categories={a:{c:[0,0] for c in ['suspended','floor_intersecting','base_on_floor','unidentified']} for a in counts}
pixels=0
for i,(row,ss,ii) in enumerate(zip(result['rows'],seal['rows'],ins['rows'])):
    assert row['frame']==ss['frame']==ii['frame'] and row['time_s']==ss['time_s']==ii['time_s']
    assert sha(D/'predictions'/ss['frame'])==ss['sha256'] and sha(D/'ideal_inputs'/ii['frame'])==ii['sha256']
    with np.load(D/'predictions'/ss['frame']) as p,np.load(H/'e0_recovery_qualification'/ss['frame']) as c,np.load(R/'frames'/ss['frame']) as a:
        t=c['target_union']
        for arm in counts:
            q=p[arm+'_cells'];d2=np.sum((q[:,None,:]-t[None,:,:])**2,axis=2)
            predmin=np.min(d2,axis=1) if len(t) and len(q) else np.full(len(q),np.inf)
            tarmin=np.min(d2,axis=0) if len(t) and len(q) else np.full(len(t),np.inf)
            direct=dict(predicted=len(q),target=len(t),correct=int((predmin<=2).sum()),covered=int((tarmin<=2).sum()))
            for k,v in direct.items():assert row[arm][k]==v;counts[arm][k]+=v
            dist[arm].extend((np.sqrt(predmin)*.1).tolist())
            for cat in categories[arm]:
                tt=c[cat+'_cells'];dd=np.sum((q[:,None,:]-tt[None,:,:])**2,axis=2)
                mn=np.min(dd,axis=0) if len(q) and len(tt) else np.full(len(tt),np.inf)
                categories[arm][cat][0]+=int((mn<=2).sum());categories[arm][cat][1]+=len(tt)
            for eye in EYES:
                qq=p[eye+'_q'];zz=p[eye+'_'+arm+'_depth'];truth=a[eye+'_depth_label'].reshape(-1)[qq]
                assert np.array_equal(qq,np.load(D/'ideal_inputs'/ss['frame'])[eye+'_q'])
                good=np.isfinite(zz);err[arm].extend(abs(zz[good]-truth[good]).tolist())
                if arm=='render':pixels+=len(qq)
for arm in counts:
    agg=result['aggregate'][arm]
    assert all(agg[k]==v for k,v in counts[arm].items())
    assert agg['all_target_pixels']==pixels==4476747
    assert agg['finite_depth_pixels']==len(err[arm])
    assert np.max(abs(np.quantile(err[arm],[.5,.9,.95])-agg['conditional_depth_p50_p90_p95_m']))<1e-12
    assert abs(np.quantile(dist[arm],.9)-agg['position_p90_m'])<1e-12
    assert counts[arm]['target']==11652
check=dict(frames=110,eyes=440,all_rows_and_aggregate_counts_exact=True,depth_quantiles_exact=True,coarse_position_quantile_exact=True,all_input_prediction_files_sha256_match=True,all_target_pixels=pixels,category_coverage=categories,wall_s=time.monotonic()-start,scope='Independent integer-grid brute-force nearest distances and full saved-depth quantiles; no generation rerun. Categories can share a cell; their counts are separate.')
(D/'independent_check.json').write_text(json.dumps(check,indent=2)+'\n')

frame='frame_055.npz'
with np.load(R/'frames'/frame) as a,np.load(D/'ideal_inputs'/frame) as lab,np.load(D/'predictions'/frame) as pred,np.load(H/'e0_recovery_qualification'/frame) as cache:
    fig,axs=plt.subplots(4,5,figsize=(17,11),constrained_layout=True)
    cmap=plt.get_cmap('viridis').copy();cmap.set_bad('#555555')
    ecmap=plt.get_cmap('magma').copy();ecmap.set_bad('#555555')
    for j,eye in enumerate(EYES):
        rgb=a[eye+'_rgb'];axs[j,0].imshow(rgb);axs[j,1].imshow(rgb)
        uv=lab[eye+'_uv'];axs[j,1].scatter(uv[:,0],uv[:,1],s=10,c='#ffad00',marker='x')
        qq=pred[eye+'_q'];zz=pred[eye+'_native_depth'];dense=np.full((480,640),np.nan);dense.reshape(-1)[qq]=zz
        truth=a[eye+'_depth_label'].copy();truth[(truth<.5)|(truth>3)|cache[eye+'_CAD']]=np.nan
        error=np.full((480,640),np.nan);good=np.isfinite(zz);error.reshape(-1)[qq[good]]=abs(zz[good]-a[eye+'_depth_label'].reshape(-1)[qq[good]])
        axs[j,2].imshow(dense,cmap=cmap,vmin=.5,vmax=3)
        axs[j,3].imshow(truth,cmap=cmap,vmin=.5,vmax=3)
        axs[j,4].imshow(error,cmap=ecmap,vmin=0,vmax=.3)
        axs[j,0].set_ylabel(eye)
        for ax in axs[j]:ax.set_xticks([]);ax.set_yticks([])
        labels=['Original RGB','Ideal tangent annotations','Ideal-contour depth / native pose','Rendered depth label','Absolute depth error (finite only)']
        for k,label in enumerate(labels):
            if j==0:axs[j,k].set_title(label,fontsize=10)
        axs[j,4].text(.02,.96,f'Output {good.sum()}/{len(qq)} target pixels',transform=axs[j,4].transAxes,color='white',va='top',fontsize=8)
    fig.colorbar(plt.cm.ScalarMappable(norm=Normalize(.5,3),cmap=cmap),ax=axs[:,2:4],label='Optical Z (m)',shrink=.7)
    fig.colorbar(plt.cm.ScalarMappable(norm=Normalize(0,.3),cmap=ecmap),ax=axs[:,4],label='Error (m), saturated above .3',shrink=.7)
    fig.suptitle('E0 fixed middle frame 55, 27.5s | IDEAL contours/families/instance IDs/class masks, NOT RGB inference\nGray: unavailable/CAD/outside displayed domain. Ground false positives and long-term memory: N/A.',fontsize=12)
    fig.savefig(D/'fixed_middle_RGB_contour_depth.png',dpi=135);plt.close(fig)
    origin=pred['origin'];root=a['root_position_physics_label'];target=cache['target_union']*.1+origin-root[:2]
    fig,axs=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
    arrays=[('Full visible target',cache['target_union']),('Ideal contours + rendered pose',pred['render_cells']),('Ideal contours + causal native pose',pred['native_cells'])]
    limit=np.max(abs(np.concatenate([target,pred['render_cells']*.1+origin-root[:2],pred['native_cells']*.1+origin-root[:2]])))+.3
    for ax,(title,pts) in zip(axs.flat,arrays):
        ax.scatter(target[:,0],target[:,1],s=30,c='#bbbbbb',marker='s',label='Target')
        xy=pts*.1+origin-root[:2];ax.scatter(xy[:,0],xy[:,1],s=12,c='#1558bd',label='Set')
        ax.plot(0,0,'r+');ax.set_aspect('equal');ax.set_xlim(-limit,limit);ax.set_ylim(-limit,limit);ax.set_title(title);ax.set_xlabel('X relative to rendered root (m)');ax.set_ylabel('Y (m)');ax.legend(fontsize=8)
    ax=axs[1,1];tt=[r['time_s'] for r in result['rows']]
    for arm,name in [('render','Rendered pose'),('native','Causal native pose')]:ax.plot(tt,[100*r[arm]['recall'] for r in result['rows']],label=name)
    ax.axhline(80,c='red',ls='--',label='80% gate');ax.set_ylim(-2,102);ax.set_xlabel('Time (s), all110 frames');ax.set_ylabel('Full target recall (%)');ax.legend(fontsize=8);ax.grid(alpha=.3)
    fig.suptitle('Fixed middle BEV and complete-sequence coverage | ideal annotations only\nMiddle target89 cells; rendered81/89 covered; native52/89. Unknown is not free; no long-term map/navigation.',fontsize=11)
    fig.savefig(D/'full_sequence_BEV_coverage.png',dpi=140);plt.close(fig)
print(json.dumps(check),flush=True)
