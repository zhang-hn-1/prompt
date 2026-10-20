from pathlib import Path
import json,numpy as np,hashlib
D=Path(__file__).resolve().parent;out=D/'inference_inputs';out.mkdir(exist_ok=False);rows=[]
for name in ['H','E0','E1']:
 a=np.load(D/'data'/f'{name}.npz');p=out/f'{name}.npz';np.savez_compressed(p,rgb=a['rgb'],gravity=a['gravity']);rows.append(dict(name=name,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),keys=['rgb','gravity']))
(D/'inference_inputs.json').write_text(json.dumps(rows,indent=2)+'\n')
