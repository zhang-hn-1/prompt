"""Closed-source development replay before any labels; no candidate truth feeds."""
from pathlib import Path
import numpy as np,json,datetime,hashlib
from qualified_hazard_consumer import QualifiedHazardConsumer,QualifiedHazardGrid
from past_point_rgb_memory import PastPointRGBMemory
H=Path(__file__).resolve().parent;O=H/'observation_handoff_observe_run_recovery_v2';out=H/'qualified_hazard_development_replay_v1';assert not out.exists();out.mkdir();states=json.loads((O/'states.json').read_text());memory=PastPointRGBMemory();c=QualifiedHazardConsumer();rows=[];inputsha={};sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):inputsha[str(p.resolve())]=sha(p);return np.load(p)
for f in sorted((O/'sensor_packets').glob('*.npz')):
 source=load(f);i=int(source['frame']);t=float(source['time_s']);rgb=load(O/'online_inputs'/f'frame_{i:03d}.npz');pose=load(O/'known_poses'/f'frame_{i:03d}.npz');mask=load(O/'masks'/f'frame_{i:03d}.npz');count=int(((source['world'][:,2]>.05)&(source['world'][:,2]<=1.5)).sum());low=i>=4 and all(np.linalg.norm(v['velocity'][:2])<.05 for v in states[i-4:i+1]);q=18-1e-6<=t<24-1e-6 and low and count>=100 and t-float(source['time_s'])<=.5
 if not memory.seeded and q:memory.seed(source,rgb,t,qualified=True)
 proof=memory.query(rgb,pose,mask,t);current,past=c.consume(source,proof,t,q);np.savez_compressed(out/f'current_{i:03d}.npz',**current);np.savez_compressed(out/f'past_{i:03d}.npz',**past);np.savez_compressed(out/f'proof_{i:03d}.npz',**proof);rows.append(dict(frame=i,**c.diagnostics))
(out/'records.json').write_text(json.dumps(rows,indent=2)+'\n');files=list(out.iterdir());(out/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputsha,output_sha256={str(p.resolve()):sha(p) for p in files},code_sha256=sha(__file__),states_sha256=sha(O/'states.json'),labels_not_read=True,original_policy_not_called=True,source_only_development_not_fresh=True),indent=2)+'\n');print('closed source-only outputs sealed',len(rows));print([(r['frame'],r['current_hazards'],r['qualified_current_hazards'],r['old_appearance_hazards']) for r in rows if 180<=r['frame']<=215])
