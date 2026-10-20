from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;W=D.parents[1];r=json.loads((D/'result.json').read_text());old=json.loads((W/'experiments/dual_monocular_semantic_bev_20261015_v1/result.json').read_text());md=0.;n=0
fig,ax=plt.subplots(7,2,figsize=(13,18),sharex=True,sharey=True,constrained_layout=True)
for k,(name,g) in enumerate(r['groups'].items()):
 for j in range(2):
  rows=g['frames'];prior=old['groups'][name]['frames'];assert len(rows)==len(prior)==181
  for a,b in zip(rows,prior):
   assert a['frame']==b['frame'];e=a['eyes'][j];oe=b['eyes'][j];assert e['valid']==oe['valid_true_pixels'];assert e['finite']==oe['finite_prediction_pixels'];assert e['over015']+e['under015']+e['within015']==e['finite'];md=max(md,abs(e['abs_p90']-oe['depth_p90_m']));n+=1
  assert sum(a['eyes'][j]['valid'] for a in rows)==old['groups'][name]['depths'][j]['true_domain']
  t=np.arange(181)*.5;q=np.array([a['eyes'][j]['ratio_q'] for a in rows]);ax[k,j].fill_between(t,q[:,0],q[:,2],alpha=.25,label='within-image p10-p90');ax[k,j].plot(t,q[:,1],label='within-image median');ax[k,j].axhline(1,color='k',linestyle='--');ax[k,j].set_title(name+' '+['left','right'][j]);ax[k,j].set_ylim(0,3.5);ax[k,j].set_ylabel('predicted / GT depth');ax[k,j].grid(alpha=.2)
assert md<1e-6
ax[0,0].legend();ax[-1,0].set_xlabel('saved source time (s)');ax[-1,1].set_xlabel('saved source time (s)');fig.suptitle('Sealed v77: full 7 development sequences, no fitting/rescaling\nAll GT-finite 0.5-3m non-CAD pixels; bands are within-image distributions, not confidence intervals')
fig.savefig(D/'ratio_sequences.png',dpi=130);plt.close(fig)
(D/'check.json').write_text(json.dumps(dict(frame_eye_comparisons=n,all_domains_and_finite_counts_exact=True,max_abs_p90_difference_vs_prior=md,all_count_partitions_exact=True,check_scope='Independent existing v77 evaluator comparison, not fresh independent model validation'),indent=2)+'\n')
print('checked',n,'max p90 delta',md)
for name,g in r['groups'].items():print(name,[(round(e['under015']/e['finite']*100,2),round(e['over015']/e['finite']*100,2),[round(v,3) for v in e['median_ratio_frame_min_median_max']]) for e in g['eyes']])
