"""Inspect frozen saved outputs only; plane labels never change predictions."""
from pathlib import Path
import json,datetime
import numpy as np
from PIL import Image,ImageDraw
H=Path(__file__).resolve().parent;o=H/'matte_coded_board_run';manifest=json.loads((o/'v1_sparse/manifest.json').read_text());assessment=json.loads((o/'v1_sparse/assessment.json').read_text());rows=[];panels=[]
for record in manifest['rows']:
 i=record['frame'];r=np.load(record['path']);labels=np.load(o/'v1_sparse'/f'labels_{i:03d}.npz');d=r['depths_m'];q=r['anchors_yx'];truth=labels['expected_depth_m'];nearest=np.abs(truth[:,None]-d).argmin(1);j=np.arange(len(q));finite=np.isfinite(r['raw_depth_m']);winner=r['best_index'];best=r['cost'][j,winner];gtcost=r['cost'][j,nearest];gtbits=r['view_bits'][j,nearest];winnerbits=r['view_bits'][j,winner]
 row=dict(frame=i,queries=len(q),raw=int(finite.sum()),retained=int(r['confidence'].sum()),correct=int(labels['correct'].sum()),no_raw=int((~finite).sum()),wide_interval=int((finite&(r['interval_ray_width_m']>.1+1e-8)).sum()),high_cost=int((finite&(best>.2)).sum()),boundary_winner=int((finite&((winner==0)|(winner==len(d)-1))).sum()),gt_nearest_valid=int(np.isfinite(gtcost).sum()),gt_nearest_current_right_valid=int(((gtbits&1)!=0).sum()),winner_has_current_right=int(((winnerbits&1)!=0).sum()),winner_any_past=int((winnerbits>1).sum()),retained_any_past=int((r['confidence']&(winnerbits>1)).sum()),rejection_counts_overlap=True)
 rows.append(row)
 frame=np.load(o/'frames'/f'frame_{i:03d}.npz');panel=Image.fromarray(frame['left_rgb']);draw=ImageDraw.Draw(panel)
 for (y,x),valid,correct in zip(q,r['confidence'],labels['correct']):
  color=(0,255,0) if correct else (255,50,50) if valid else (255,210,0);draw.ellipse((int(x)-4,int(y)-4,int(x)+4,int(y)+4),outline=color,width=2)
 top=Image.new('RGB',(640,510),'white');top.paste(panel,(0,30));ImageDraw.Draw(top).text((7,8),f'{i:03d} | green correct / red retained wrong / yellow rejected | {row["retained"]}/24',fill='black');panels.append(top)
fig=Image.new('RGB',(1280,1530),'white')
for n,panel in enumerate(panels):fig.paste(panel,((n%2)*640,(n//2)*510))
fig.save(H/'figures/matte_board_v1_all_anchors.png')
(o/'v1_sparse/failure_partition.json').write_text(json.dumps(dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,scope='savedV1 output-only plus AFTER-prediction board-plane labels; nearest grid support not exacttrue-depth complete source visibility; no predictor/model/gate change'),indent=2)+'\n')
print(json.dumps(rows,indent=2))
