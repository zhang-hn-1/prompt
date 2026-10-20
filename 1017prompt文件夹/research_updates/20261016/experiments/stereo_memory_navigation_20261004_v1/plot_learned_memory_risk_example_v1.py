"""Post-seal offline-labelled illustration, no control/model inputs or new evaluation gate."""
from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from learned_memory_risk_dataset_v1 import offline_teacher_labels,sha
H=Path(__file__).resolve().parent;name='rolling_endpoint_coverage_native_e2_run_v1';D=H/name
if __name__=='__main__':
 assert (H/'learned_memory_risk_offline_closed_review_v1.json').exists()
 pred=np.load(H/(name+'_risk_predictions_v1.npz'));meta=json.loads((H/(name+'_risk_predictions_v1.json')).read_text());y,h,inputs=offline_teacher_labels(D,meta['index']);keep=pred['keep'];frames=pred['frame']
 counts={int(f):int(((frames==f)&(y==0)).sum()) for f in np.unique(frames)};frame=max(counts,key=counts.get);selected=frames==frame;rgb=np.load(D/'source_inputs'/f'frame_{frame:03d}.npz');record=np.load(D/'partial_observations'/f'frame_{frame:03d}.npz');ids=[record['query_pixel_id'],record['partner_pixel_id']];labels=y[selected];kept=keep[selected]
 fig,axes=plt.subplots(1,2,figsize=(14,5))
 groups=[(labels==0)&(~kept),(labels==0)&kept,(labels==1)&(~kept)]
 for ax,key,pixels in zip(axes,['left_rgb','right_rgb'],ids):
  ax.imshow(rgb[key]);
  for mask,color,marker,label in zip(groups,['cyan','orange','red'],['x','o','+'],['floor risk removed -> UNKNOWN','floor risk retained','hazard risk removed']):
   ax.scatter(pixels[mask]%640,pixels[mask]//640,c=color,marker=marker,s=45,facecolors='none' if marker=='o' else None,label=label)
  ax.set_title(key);ax.set_xlim(0,639);ax.set_ylim(479,0);ax.set_axis_off()
 axes[0].legend(loc='upper left',fontsize=8);fig.suptitle(f'Offline teacher-labelled example: frame {frame}, selected for most floor query points\nOriginal physical run was unchanged; this does not show an online learned navigator',fontsize=11);fig.tight_layout();out=H/'learned_memory_risk_offline_example_v1.png';fig.savefig(out,dpi=150);plt.close(fig)
 (H/'learned_memory_risk_offline_example_identity_v1.json').write_text(json.dumps(dict(frame=frame,time_s=float(rgb['time_s']),selected_for_maximum_floor_points=True,counts=dict(floor=int((labels==0).sum()),floor_removed=int(groups[0].sum()),floor_retained=int(groups[1].sum()),hazard_removed=int(groups[2].sum())),raw_input_sha256={str(D/'source_inputs'/f'frame_{frame:03d}.npz'):sha(D/'source_inputs'/f'frame_{frame:03d}.npz')},not_online=True,no_control_change=True,not_average_performance=True,output_sha256=sha(out)),indent=2)+'\n');print(frame)
