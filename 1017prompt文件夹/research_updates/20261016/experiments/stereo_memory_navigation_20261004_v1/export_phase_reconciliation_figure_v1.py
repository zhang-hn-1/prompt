from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;W=H.parent.parent
p=H/'research_phase_reconciliation_assessment_v2.json';j=json.loads(p.read_text())
fig,axes=plt.subplots(1,2,figsize=(12,4.8),sharey=True)
for ax,layer,title in zip(axes,('known_pose','estimated_pose'),('Known pose (privileged diagnosis)','Estimated pose (initial gauge + ideal IMU)')):
 for off,variant,color,label in [(-.18,'frozen_classical','#687a94','Frozen classical'),(.18,'rolling_endpoint','#13898b','Rolling endpoint')]:
  rows=[r for r in j['records'] if r['pose_layer']==layer and r['variant']==variant]
  for i,r in enumerate(rows):
   ax.bar(i+off,r['net_axis_progress_m'],width=.33,color=color,label=label if i==0 else None)
   ax.annotate(r['outcome'],(i+off,r['net_axis_progress_m']),xytext=(0,4),textcoords='offset points',ha='center',fontsize=8,rotation=25)
 ax.set_xticks(range(3),['E0','E1','E2']);ax.set_title(title,fontsize=10);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True);ax.legend(fontsize=8,loc='upper right')
axes[0].set_ylabel('Measured net progress on frozen start-goal axis (m)')
fig.suptitle('TaskH navigation evidence: each condition measured once in its registry',fontsize=12)
fig.text(.5,.015,'Fixed camera/light; 35 s rule. Conditions are separated, not pooled. No causal improvement or pure-RGB safety claim.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.05,1,.94));out=H/'research_phase_reconciliation_progress_v1.png';assert not out.exists();fig.savefig(out,dpi=180);plt.close(fig)
(H/'research_phase_reconciliation_export_identity_v1.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [p,Path(__file__).resolve(),H/'reconcile_research_phases_v2.py',out]},indent=2)+'\n')
