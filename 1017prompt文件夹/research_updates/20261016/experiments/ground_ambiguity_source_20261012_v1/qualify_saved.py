from pathlib import Path
import sys,json,hashlib,time,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';H=W/'experiments/stereo_memory_navigation_20261004_v1';sys.path[:0]=[str(D),str(V),str(H),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from ground_ambiguity import qualify_prediction
from rig_adapter import Rig,RigMask
R=V/'candidate_e2_run';states=json.loads((R/'states.json').read_text());cad=RigMask(Rig('forward',.4,.3,.05));out=D/'qualified_predictions';out.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();sources={};rows=[]
for p in sorted((R/'source_predictions').glob('*.npz')):
 i=int(p.stem.split('_')[1]);rgb=np.load(R/'source_inputs'/p.name);poses=dict(np.load(R/'known_poses'/p.name));pred=dict(np.load(p));assert float(pred['time_s'])==float(rgb['time_s'])==float(poses['time_s']);mask=cad.project(np.array(states[i]['joints']));t=time.monotonic();v=qualify_prediction(pred,rgb['left_rgb'],rgb['right_rgb'],poses,float(pred['time_s']),mask);np.savez_compressed(out/p.name,**v);rows.append(dict(frame=i,ambiguous=v['ground_ambiguity_count'],wall_s=time.monotonic()-t))
 for q in [p,R/'source_inputs'/p.name,R/'known_poses'/p.name]:sources[str(q)]=sha(q)
 if i%100==0:print(i,'qualified',v['ground_ambiguity_count'],flush=True)
for p in [D/'ground_ambiguity.py',D/'protocol.json',D/'qualify_saved.py',V/'rig_adapter.py',R/'states.json']:sources[str(p)]=sha(p)
seal=dict(input_sha256=sources,output_sha256={str(p):sha(p) for p in out.glob('*.npz')},rows=rows,renderer_truth_read=False)
(D/'prediction_seal.json').write_text(json.dumps(seal,indent=2)+'\n');print('prediction_seal_complete',len(rows),flush=True)
