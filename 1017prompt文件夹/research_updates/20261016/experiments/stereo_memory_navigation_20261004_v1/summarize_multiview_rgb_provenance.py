"""Offline label partition on saved winner bits, no predictor calls."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent
m=np.load(H/'multiview_rgb_measurements.npz');r=json.loads((H/'multiview_rgb_audit.json').read_text());rows=[]
for row in r['rows']:
    f=row['frame'];pre=f'frame_{f:03d}__';y,x=m[pre+'source_pixels_yx'].T
    p=np.load(H/'multiview_rgb_predictions'/f'frame_{f:03d}.npz');bits=p['view_bits'][y,x];visible=m[pre+'visible_source_bits']
    conf=m[pre+'confident'];correct=m[pre+'correct'];anyvisible=(bits&visible)!=0;allvisible=(bits!=0)&((bits&visible)==bits)
    flags=dict(confident_no_true_visible_source=conf&~anyvisible,wrong_no_true_visible_source=conf&~correct&~anyvisible,
        confident_any_true_visible_source=conf&anyvisible,confident_all_true_visible_sources=conf&allvisible,
        wrong_all_true_visible_sources=conf&~correct&allvisible)
    rows.append(dict(frame=f,**{k:int(v.sum()) for k,v in flags.items()}))
keys=[k for k in rows[0] if k!='frame'];pooled={k:sum(r[k] for r in rows) for k in keys}
(H/'multiview_rgb_provenance_summary.json').write_text(json.dumps(dict(role='offline true point visibility of saved centre winner source bits; not online filter, not full physical patch certificate',rows=rows,pooled=pooled),indent=2)+'\n')
print(pooled)
