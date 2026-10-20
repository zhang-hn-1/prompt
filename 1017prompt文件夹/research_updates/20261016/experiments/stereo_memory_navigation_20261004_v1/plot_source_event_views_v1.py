from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;D=H/'source_event_handoff_C1_run_v1';O=H/'qualified_appearance_extent_v1';s=json.loads((O/'prediction_seal_v1.json').read_text());a=json.loads((O/'assessment_v1.json').read_text());fig,axs=plt.subplots(5,2,figsize=(11,17),constrained_layout=True)
for ax,r,score in zip(axs.flat,s['rows'],a['all_10_development_rows']):
 rgb=np.load(D/'online_inputs'/f"frame_{r['frame']:03d}.npz")[r['eye']+'_rgb'][:,:,:3];m=np.load(r['file'])['potential_hazard_mask'];im=rgb.copy();im[m]=(im[m]*.6+np.array([255,0,100])*.4).astype(np.uint8);ax.imshow(np.rot90(im));ax.axis('off');ax.set_title(f"{r['eye']} frame{r['frame']} / hypothesis born15s\nprecision {score['precision']:.3f}, hazard coverage {score['coverage']:.3f}",fontsize=9)
fig.suptitle('One source-derived appearance mask (pink): hypotheses, not depth or occupancy')
fig.savefig(H/'qualified_appearance_extent_actual_v1.png',dpi=110)
