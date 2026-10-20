from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
with np.load(D/'failure_analysis_arrays.npz') as a:cost=a['cost'];total=a['total'];left=a['left_path'];right=a['right_path'];top=a['top_path']
with np.load(D/'repair_arrays.npz') as a:old=a['fixed_old'];new=a['fixed_new']
y,x=5,6;fig,ax=plt.subplots(1,4,figsize=(14,4));fig.suptitle('FIXED CONSTANT-RGB ENGINEERING FIXTURE ONLY\nActual four RGB / GT error / BEV / navigation = N/A',fontsize=14)
ax[0].bar([0,1],cost[y,x],color=['gray','orange']);ax[0].set_ylim(0,700);ax[0].set_xticks([0,1]);ax[0].set_title('Local photometric costs: [0,0]');ax[0].set_xlabel('disparity candidate')
for i,(v,n) in enumerate([(left,'left'),(right,'right'),(top,'top')]):ax[1].bar(np.array([0,1])+i*.18,v[y,x],width=.17,label=n)
ax[1].set_xticks([.18,1.18],[0,1]);ax[1].set_ylim(0,700);ax[1].legend();ax[1].set_title('Path costs: border -> prior penalty')
ax[2].bar([0,1],total[y,x],color=['gray','orange']);ax[2].set_ylim(0,700);ax[2].set_xticks([0,1]);ax[2].set_title('Aggregate [0,600]: unique index0')
ax[3].axis('off');ax[3].text(0,.9,'Original: candidate 0\n\nRepaired evidence rule: UNKNOWN\n\nLocal unique minimum required\nAND aggregate must agree\n\nIndex0 is NOT 0 meters.\nNo real obstacle result.',va='top',fontsize=12)
fig.tight_layout(rect=(0,0,1,.8));fig.savefig(D/'fixed_flat_cause_and_repair.png',dpi=140);plt.close(fig);print('SYNTHETIC_SAVED_ARRAY_RENDER_ONLY')
