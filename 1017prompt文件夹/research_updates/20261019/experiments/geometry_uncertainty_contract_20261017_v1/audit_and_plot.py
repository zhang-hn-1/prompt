"""CPU saved-array audit and RGB-only perturbation evidence; no field execution."""
from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;W=D.parents[1]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
c=json.loads((D/'protocol.json').read_text());r=json.loads((D/'result.json').read_text())
assert sha(D/'RGB_arrays.npz')==r['arrays_sha256'];hashes=1
for p,h in r['source_sha256'].items():assert sha(p)==h,p;hashes+=1
for p,h in c['checkpoint_sha256'].items():assert sha(W/p)==h,p;hashes+=1
prior=W/'experiments/radiance_pointwise_qualification_20261016_v1'
for n,h in json.loads((prior/'completed_seal.json').read_text())['files'].items():assert sha(prior/n)==h,n;hashes+=1
saved=np.load(prior/'pointwise_arrays.npz');a=np.load(D/'RGB_arrays.npz')
max_error=0.;zero_rgb_error=0.;checked=0
for name,arm in r['arms'].items():
    recomputed=[]
    for row in arm['rows']:
        key=name+'_'+row['input'][:-4];target=a[key+'_observed'].astype(np.float64)
        zero_rgb_error=max(zero_rgb_error,float(abs(a[key+'_0']-saved[key+'_rgb']).max()))
        assert np.array_equal(a[key+'_observed'],saved[key+'_targetRGB'])
        losses={label:float(np.mean(np.sum((a[key+'_'+label].astype(np.float64)-target)**2,axis=-1))) for label in ['minus','0','plus']}
        for label,x in losses.items():max_error=max(max_error,abs(x-row['RGB_SSE'][label]))
        recomputed.append(losses);checked+=1
    for aggregation,weight in [('equal_view',np.full(8,1/8)),('training_expectation',np.array(c['aggregations']['training_expectation']))]:
        agg=arm['aggregates'][aggregation]
        losses={k:float(np.dot(weight,[v[k] for v in recomputed])) for k in ['minus','0','plus']}
        for label,x in losses.items():assert abs(x-agg['RGB_SSE'][label])<1e-15
        gradient=float(np.dot(weight,[x['derivative_RGB_SSE_per_m'] for x in arm['rows']]))
        assert gradient==agg['derivative_RGB_SSE_per_m']
        assert agg['stationary_centre_contradicted'] and gradient>0
        assert losses['minus']<losses['0']-agg['corroboration_threshold']
assert checked==16 and zero_rgb_error==0 and max_error<1e-15
audit=dict(complete=True,CPU_only=True,view_checks=checked,rays_per_field=9600,hash_checks=hashes,
    delta0_RGB_matches_prior_exact=True,delta0_max_abs_RGB_error=zero_rgb_error,
    observed_samples_match_prior_exact=True,max_abs_loss_error=max_error,
    all4_aggregate_nonstationarity_decisions_confirmed=True,renderer_calls=0,optimizer_steps=0)
(D/'audit_result.json').write_text(json.dumps(audit,indent=2)+'\n')
fig,axs=plt.subplots(1,2,figsize=(11,4.8),layout='constrained')
for ax,aggregation,title in zip(axs,['equal_view','training_expectation'],['Equal view weights','Original training view weights']):
    for name,color,label in [('RGB_init_failed','#c84c37','Wrong geometry: RGB init'),('known_density_init_qualified','#3476a8','Qualified geometry: known density init*')]:
        values=r['arms'][name]['aggregates'][aggregation]['RGB_SSE']
        ax.plot([-1,0,1],[values[k]*1e3 for k in ['minus','0','plus']],marker='o',color=color,label=label)
        change=(values['0']-values['minus'])/values['0']*100
        ax.annotate(f'−1 mm: loss −{change:.2f}%',(-1,values['minus']*1e3),xytext=(7,10),textcoords='offset points',color=color,fontsize=9)
    ax.set_title(title);ax.set_xlabel('Frozen-field query offset in world X (mm)')
    ax.set_ylabel('Observed RGB SSE per ray × 1000');ax.set_xticks([-1,0,1]);ax.grid(alpha=.2);ax.margins(y=.15)
axs[0].legend(fontsize=8)
fig.suptitle('Zero offset is not an observed-RGB stationary centre in this direction\n2 frozen fields × 8 views × 1200 fixed rays; no scene update or depth correction',fontsize=11)
fig.text(.5,-.04,'* Privileged known geometry initialization; synthetic known background. No uncertainty interval, depth-error or navigation result.',ha='center',fontsize=9)
fig.savefig(D/'stationarity.png',dpi=160,bbox_inches='tight');plt.close(fig)
print(json.dumps(audit))
