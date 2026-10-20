"""Post-evaluation audit only: all 24 rejected valid pairs, no predictor changes."""
from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent; O=H/'anchor_evidence_development_v1'; D=H/'minimum_prefix_transition_navigation_run_v1'
rows=json.loads((O/'miss_height_diagnosis_v1.json').read_text())['misses']
assert len(rows)==24
summary=[]
for page in range(3):
 fig,axes=plt.subplots(8,2,figsize=(11,24),constrained_layout=True)
 for i,r in enumerate(rows[page*8:(page+1)*8]):
  q=np.asarray(r['query_xy']); s=np.asarray(r['source_xy']); t=np.asarray(r['GT_projected_source_xy']);
  for col,(frame,centre) in enumerate([(r['query_frame'],q),(r['source_frame'],s)]):
   a=np.load(D/'source_inputs'/f'frame_{frame:03d}.npz')['left_rgb']; ax=axes[i,col];ax.imshow(a)
   ax.set_xlim(centre[0]-40,centre[0]+40);ax.set_ylim(centre[1]+40,centre[1]-40)
   if col==0:
    ax.plot(*q,'r+',markersize=12);ax.set_title(f"{page*8+i+1}: current {frame}, query red +")
   else:
    ax.plot(*s,'r+',markersize=12);ax.plot(*t,'co',fillstyle='none',markersize=12)
    ax.set_title(f"past {frame}: match red +; GT projection cyan o\nmax-axis offset {np.max(np.abs(s-t)):.2f}px; {r['polytope_status']}")
  summary.append(dict(pair=r,current_archive=D.name,GT_projection_after_rejection_only=True,pixel_offset_max=float(np.max(np.abs(s-t)))))
 fig.suptitle('Historical correspondence failure diagnostic: LEFT at two times; no new model inputs',fontsize=14)
 fig.savefig(O/f'correspondence_misses_page_{page+1}.png',dpi=100);plt.close(fig)
result=dict(pair_count=len(rows),pairs=summary,all_rejected_valid_pairs_in_original_order=True,no_new_source_or_control_candidate=True,teacher_nearest_pixel_depth_at_unrounded_query=True,teacher_is_not_correspondence_certificate=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(O/'correspondence_visual_diagnosis_v1.json').write_text(json.dumps(result,indent=2)+'\n')
print('24 pairs, 3 pages, post-evaluation only')
