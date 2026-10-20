from pathlib import Path
import json,numpy as np,hashlib
D=Path(__file__).resolve().parent;A=D/'baseline_e1_run';B=D/'candidate_e1_run';rows=[]
for p in sorted((A/'source_inputs').glob('*.npz')):
 if not (B/'source_inputs'/p.name).exists():continue
 a=np.load(p);b=np.load(B/'source_inputs'/p.name);keys=[k for k in a.files if 'rgb' in k];rows.append(dict(frame=p.name,same_rgb=all(np.array_equal(a[k],b[k]) for k in keys)))
prefix=0
for r in rows:
 if not r['same_rgb']:break
 prefix+=1
out=dict(compared_pairs=len(rows),identical_rgb_pairs=sum(r['same_rgb'] for r in rows),identical_prefix_pairs=prefix,full_causal_replication=bool(rows and all(r['same_rgb'] for r in rows)),limits='Same frozen E1 conditions but differing actual RGB prevents full paired input causal claim; development scene not generalization.')
(D/'pair_identity.json').write_text(json.dumps(out,indent=2)+'\n');print(out)
