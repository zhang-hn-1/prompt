from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
with np.load(D/'reference_arrays.npz') as a:arr={k:a[k] for k in a.files}
fig,axes=plt.subplots(2,3,figsize=(11,5))
for ax,key,title in zip(axes.ravel(),['unmasked_0_winner','unmasked_100000_winner','masked_0_winner','masked_100000_winner','allmissing_winner','gap_winner'],['UNMASKED: invalid cost=0','UNMASKED: invalid cost=100000','MASKED: invalid cost=0','MASKED: invalid cost=100000','ALL MISSING: unknown','GAP: restart after unknown']):
 v=arr[key];ax.imshow(v,vmin=-1,vmax=3,cmap='viridis',aspect='auto')
 for y,x in np.ndindex(v.shape):ax.text(x,y,'?' if v[y,x]<0 else str(v[y,x]),ha='center',va='center',color='white',fontsize=12)
 ax.set_title(title,fontsize=10);ax.set_xticks([]);ax.set_yticks([])
fig.suptitle('SYNTHETIC COST VOLUMES ONLY: selected index, ? = unknown\nActual four RGB / metric error / BEV / navigation: N/A',fontsize=13)
fig.tight_layout(rect=(0,0,1,.87));fig.savefig(D/'synthetic_support_contract.png',dpi=140);plt.close(fig)
print('SYNTHETIC_RENDER_NO_KERNEL_OR_RGB_CALLS')
