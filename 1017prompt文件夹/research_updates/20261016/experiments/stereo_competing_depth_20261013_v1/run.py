from pathlib import Path
import sys,json,hashlib,os
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];C=W/'experiments/source_confidence_calibration_20261012_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';H=W/'experiments/stereo_memory_navigation_20261004_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(D),str(V),str(H),str(P)];from competing import qualify
from rig_adapter import Rig,RigMask
cad=RigMask(Rig('forward',.4,.3,.05));output=D/'predictions';output.mkdir(exist_ok=False);inputs={};rows=[]
for name,R in [('H',W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run'),('E0',C/'data_e0_run'),('E1',C/'data_e1_run')]:
 states=json.loads((R/'states.json').read_text());rows.append(dict(name=name,directory=str(R)))
 for p in sorted((R/'source_predictions').glob('*.npz')):
  i=int(p.stem.split('_')[1]);a=dict(np.load(p));b=dict(np.load(R/'source_inputs'/p.name));packet=dict(np.load(R/'sensor_packets'/p.name.replace('frame_','source_')));m=cad.project(np.array(states[i]['joints']));v=qualify(b,a,packet,m,float(a['time_s']));np.savez_compressed(output/f'{name}_{p.name}',**v)
  for pp in [p,R/'source_inputs'/p.name,R/'sensor_packets'/p.name.replace('frame_','source_')]:inputs[str(pp)]=hashlib.sha256(pp.read_bytes()).hexdigest()
  if i%100==0:print(name,i,'rejected',int((~v['keep']).sum()),flush=True)
 inputs[str(R/'states.json')]=hashlib.sha256((R/'states.json').read_bytes()).hexdigest()
seal=dict(input_sha256=inputs,output_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in output.glob('*.npz')},source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'protocol.json',D/'competing.py',D/'run.py']},rows=rows,truth_read=False,fit=0);(D/'prediction_seal.json').write_text(json.dumps(seal,indent=2)+'\n')
# Score only after predictions sealed. Finite-only metric and separate infinity counts.
results=[]
for r in rows:
 R=Path(r['directory']);name=r['name'];e0=[];e1=[];ob=0;obr=0;n=0;nr=0;inf=0;infr=0
 for p in sorted((R/'source_predictions').glob('*.npz')):
  a=np.load(p);v=np.load(output/f'{name}_{p.name}');q=v['query'];k=v['keep'];f=np.load(R/'frames'/p.name);x=q%640;y=q//640;gt=f['left_depth_m'][y,x];z=a['depth_m'][y,x];ok=np.isfinite(gt)&(gt>0);e0.extend(np.abs(z[ok]-gt[ok]));e1.extend(np.abs(z[ok&k]-gt[ok&k]));T=f['left_camera_usd_world'];ray=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1);truth=T[:3,3]+(ray*np.where(ok,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;obs=ok&(truth[:,2]>.05)&(truth[:,2]<=1.5);ob+=int(obs.sum());obr+=int((obs&k).sum());n+=len(k);nr+=int(k.sum());inf+=int(np.isposinf(gt).sum());infr+=int((np.isposinf(gt)&k).sum())
 e0=np.array(e0);e1=np.array(e1);r=dict(sequence=name,candidates=n,retained=nr,before_p90_m=float(np.quantile(e0,.9)),after_p90_m=float(np.quantile(e1,.9)),above030_fraction=float((e1>.3).mean()),obstacle_retention=obr/max(1,ob),candidate_retention=nr/max(1,n),infinite_truth_candidates=inf,infinite_truth_retained=infr);r['source_gate']=bool(r['after_p90_m']<=.15 and r['above030_fraction']<=.1 and r['obstacle_retention']>=.95 and r['candidate_retention']>=.7);results.append(r)
final=dict(results=results,pass_gate=all(x['source_gate'] for x in results),fit=0,physics=0,scope='current paired competitor mechanism development source-only; no navigation increment');(D/'result.json').write_text(json.dumps(final,indent=2)+'\n');print(json.dumps(final,indent=2),flush=True)
