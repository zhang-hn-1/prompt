"""Historical17-packet preflight; seal all RGB-only outputs before labels."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from bilateral_patch_confidence import filter_packet
H=Path(__file__).resolve().parent;D=H/'controlled_heading_probe_run_v1';O=H/'bilateral_patch_confidence_v1';O.mkdir(exist_ok=False);sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();inputs={};rows=[]
for f in sorted((D/'sensor_packets').glob('source_*.npz')):
 p=np.load(f);i=int(p['frame']);t=float(p['time_s']);rf=D/'online_inputs'/f'frame_{i:03d}.npz';mf=D/'masks'/f'frame_{i:03d}.npz';rgb=np.load(rf);mask=np.load(mf);assert float(rgb['time_s'])==float(mask['time_s'])==t
 for a in [f,rf,mf]:inputs[str(a)]=sha(a)
 q,d=filter_packet(p,rgb['left_rgb'],rgb['right_rgb'],mask['left'],mask['right'],t)
 np.savez_compressed(O/f'source_{i:03d}.npz',**q);rows.append(dict(frame=i,**d))
for f in [Path(__file__),H/'bilateral_patch_confidence.py',H.parent.parent/'docs/superpowers/plans/2026-10-07-bilateral-patch-confidence.md']:inputs[str(f)]=sha(f)
manifest=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),historical_development_not_fresh=True,GT_labels_not_read=True,extra_NN_calls=0,extra_physics_sequences=0,input_sha256=inputs,output_sha256={str(f):sha(f) for f in O.glob('source_*.npz')},diagnostics=rows,whole_research_complete=False)
(O/'prediction_seal_v1.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(rows,indent=2))
