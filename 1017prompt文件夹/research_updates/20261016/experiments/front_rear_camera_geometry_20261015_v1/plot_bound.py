from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;r=json.loads((D/'coverage_upper_bound.json').read_text());a=np.load(D/'coverage_example.npz');fig,ax=plt.subplots(1,2,figsize=(13,5));names=list(r['groups']);vals=[100*r['groups'][g]['retention_upper_bound'] for g in names];ax[0].bar(names,vals,color='steelblue');ax[0].axhline(95,c='red',ls='--',label='Frozen original-target retention gate 95%');ax[0].set_ylim(0,103);ax[0].set_ylabel('Original visible target cell instances retained (%)');ax[0].legend(fontsize=8)
for i,v in enumerate(vals):ax[0].text(i,v+1,f'{v:.1f}',ha='center')
o=a['origin'];t=a['target_cells']*.1+o;c=a['retained_cells']*.1+o;ax[1].scatter(t[:,0],t[:,1],c='red',s=22,label='Original visible GT obstacle target');ax[1].scatter(c[:,0],c[:,1],c='green',s=10,label='Candidate no-occlusion upper bound');ax[1].axis('equal');ax[1].legend(fontsize=8);ax[1].set_title('B65 initial frame; fixed first pose, not worst\nGT-only geometry, blank unknown; no permanent map')
fig.suptitle('1267 archived complete frames; calibrated GT parent consensus\nNOT new-view rendered union recall, monocular precision or navigation');fig.tight_layout();fig.savefig(D/'coverage_bound.png',dpi=160)
