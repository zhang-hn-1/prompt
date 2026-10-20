from pathlib import Path
import sys,json,hashlib
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];C=W/'experiments/source_confidence_calibration_20261012_v1'
from ownership import qualify
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
output=D/'predictions';output.mkdir(exist_ok=False);inputs={};rows=[]
for name,R in [('H',W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run'),('E0',C/'data_e0_run'),('E1',C/'data_e1_run')]:
 termination=json.loads((R/'termination.json').read_text());assert termination['termination'] in ['timeout90','collision','arrival','goal_reached']
 paths=sorted((R/'source_predictions').glob('*.npz'));assert len(paths)==181
 rows.append(dict(name=name,directory=str(R),termination=termination,epochs=len(paths)))
 for p in paths:
  pp=R/'sensor_packets'/p.name.replace('frame_','source_');a=dict(np.load(pp));v=qualify(a,float(np.load(p)['time_s']));np.savez_compressed(output/f'{name}_{p.name}',**v)
  inputs[str(pp)]=sha(pp);inputs[str(p)]=sha(p)
 inputs[str(R/'termination.json')]=sha(R/'termination.json')
 print(name,'complete causal outputs',len(paths),flush=True)
seal=dict(input_sha256=inputs,output_sha256={str(p):sha(p) for p in output.glob('*.npz')},source_sha256={str(p):sha(p) for p in [D/'protocol.json',D/'ownership.py',D/'run.py']},rows=rows,truth_read=False,fit=0)
(D/'prediction_seal.json').write_text(json.dumps(seal,indent=2)+'\n')
results=[];truth_hashes={}
for r in rows:
 R=Path(r['directory']);name=r['name'];errors=[[],[]];n=nr=ob=obr=inf=infr=0
 for p in sorted((R/'source_predictions').glob('*.npz')):
  a=np.load(p);v=np.load(output/f'{name}_{p.name}');q=v['query'];k=v['keep'];fp=R/'frames'/p.name;f=np.load(fp);truth_hashes[str(fp)]=sha(fp);x=q%640;y=q//640;gt=f['left_depth_m'][y,x];z=a['depth_m'][y,x];ok=np.isfinite(gt)&(gt>0)
  errors[0].append(np.abs(z[ok]-gt[ok]));errors[1].append(np.abs(z[ok&k]-gt[ok&k]))
  T=f['left_camera_usd_world'];ray=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1);truth=T[:3,3]+(ray*np.where(ok,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;obs=ok&(truth[:,2]>.05)&(truth[:,2]<=1.5);ob+=int(obs.sum());obr+=int((obs&k).sum());n+=len(k);nr+=int(k.sum());inf+=int(np.isposinf(gt).sum());infr+=int((np.isposinf(gt)&k).sum())
 e0,e1=[np.concatenate(e) for e in errors];r=dict(sequence=name,candidates=n,retained=nr,rejected=n-nr,finite_before=len(e0),finite_after=len(e1),conditional_scoring_coverage_before=len(e0)/n,conditional_scoring_coverage_after=len(e1)/max(1,nr),full_image_scored_fraction_after=len(e1)/(181*640*480),before_percentiles=np.quantile(e0,[.5,.9,.95]).tolist(),after_percentiles=np.quantile(e1,[.5,.9,.95]).tolist(),above030_fraction=float((e1>.3).mean()),obstacle_before=ob,obstacle_after=obr,obstacle_retention=obr/max(1,ob),candidate_retention=nr/max(1,n),infinite_truth_before=inf,infinite_truth_after=infr)
 r['source_gate']=bool(len(e1)>=1000 and r['after_percentiles'][1]<=.15 and r['above030_fraction']<=.1 and r['obstacle_retention']>=.95 and r['candidate_retention']>=.7);results.append(r);print(r,flush=True)
(D/'truth_scoring_sha.json').write_text(json.dumps(truth_hashes,indent=2)+'\n')
(D/'result.json').write_text(json.dumps(dict(results=results,pass_gate=all(r['source_gate'] for r in results),physics=0,fit=0,scope='development source packet ownership; no navigation'),indent=2)+'\n')
