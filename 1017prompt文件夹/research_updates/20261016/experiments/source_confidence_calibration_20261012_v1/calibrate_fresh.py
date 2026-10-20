from pathlib import Path
import json,hashlib,time
import numpy as np,cv2
from scipy.optimize import minimize
from scipy.special import expit
D=Path(__file__).resolve().parent;W=D.parents[1]
H=W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1/candidate_e2_run'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
from features import features
hs=json.loads((H/'states.json').read_text());vs=json.loads((V/'states.json').read_text());diff=[i for i,(h,v) in enumerate(zip(hs,vs)) if np.linalg.norm(np.array(h['position'])-v['position'])>1e-5];first=diff[0] if diff else 901;cut=first*.1+10
sources={};coverage=[];packs={}
for ep in ['e0','e1']:
 auditfile=D/f'data_{ep}_raw_audit.json';audit=json.loads(auditfile.read_text());assert audit['pass_audit']
 statusfile=D/f'data_{ep}_status.json';status=json.loads(statusfile.read_text());assert status['status']=='finished' and status['child_returncode']==0
 sealfile=D/f'data_{ep}_run/artifact_manifest_post_close.json';seal=json.loads(sealfile.read_text());assert seal['execution_status']=='finished' and seal['returncode']==0
 for file,h in seal['artifact_sha256'].items():assert digest(Path(file))==h,file
 for file in [auditfile,statusfile,sealfile,D/'fresh_protocol.json',D/'calibrate_fresh.py',D/'features.py',D/'confidence.py']:sources[str(file.resolve())]=digest(file)
for name,R in [('H',H),('E0',D/'data_e0_run'),('E1',D/'data_e1_run')]:
 rows=[];Xs=[];Ys=[];Zs=[];Ts=[];Obs=[];Frames=[];Q=[]
 predpaths=sorted((R/'source_predictions').glob('*.npz'));expected=list(range(0,len(json.loads((R/'states.json').read_text())),5));assert [int(p.stem.split('_')[1]) for p in predpaths]==expected
 states=json.loads((R/'states.json').read_text());term=json.loads((R/'termination.json').read_text());assert term['termination'] in ['timeout90','success','collision'];assert 0<float(states[-1]['time_s'])<=90.0001;assert abs(float(states[-1]['time_s'])-float(term['time_s']))<1e-5;assert term['actions']==len(states)-1;assert term['termination']!='timeout90' or abs(float(states[-1]['time_s'])-90)<1e-4
 for pp in predpaths:
  i=int(pp.stem.split('_')[1]);paths=[pp,R/'source_inputs'/pp.name,R/'sensor_packets'/pp.name.replace('frame_','source_'),R/'frames'/pp.name]
  pred=dict(np.load(paths[0]));rgb=dict(np.load(paths[1]));packet=dict(np.load(paths[2]));assert abs(float(pred['time_s'])-float(rgb['time_s']))<1e-7;assert abs(float(pred['time_s'])-float(packet['time_s']))<1e-7
  X,q=features(rgb,pred,packet);f=np.load(paths[3]);assert abs(float(f['time_s'])-float(pred['time_s']))<1e-7;y=q//640;x=q%640;gt=f['left_depth_m'][y,x];ok=np.isfinite(gt)&(gt>0);z=pred['depth_m'][y,x];label=np.abs(z-gt)>.15
  T=f['left_camera_usd_world'];rays=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1);gtworld=T[:3,3]+(rays*np.where(ok,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;obstacle=ok&(gtworld[:,2]>.05)&(gtworld[:,2]<=1.5)
  rows.append(dict(frame=i,t=float(pred['time_s']),candidates=len(q),labels=int(ok.sum()),bad=int((label&ok).sum())))
  Xs.append(X);Ys.append(np.where(ok,label,-1));Zs.append(z);Ts.append(gt);Obs.append(obstacle);Frames.append(np.full(len(q),i));Q.append(q)
  for p in paths:
   sources[str(p)]=digest(p)
   if name=='H':
    hseal=json.loads((H/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];assert sources[str(p)]==hseal[str(p)],p
  if i%100==0:print(name,i,flush=True)
 X=np.concatenate(Xs);Y=np.concatenate(Ys);fr=np.concatenate(Frames);eligible=[r for r in rows if r['candidates']>=20];fraction=sum(r['labels'] for r in rows)/max(1,sum(r['candidates'] for r in rows));epochfrac=sum(r['labels']>=20 for r in eligible)/max(1,len(eligible));suffix=[r for r in rows if r['t']>cut]
 suffixfraction=sum(r['labels'] for r in suffix)/max(1,sum(r['candidates'] for r in suffix));suffixeligible=[r for r in suffix if r['candidates']>=20];suffixepochfrac=sum(r['labels']>=20 for r in suffixeligible)/max(1,len(suffixeligible));passed=fraction>=.70 and epochfrac>=.95 and sum(r['labels'] for r in rows)>=1000;coverage.append(dict(sequence=name,rows=rows,label_fraction=fraction,eligible_epoch_label_fraction=epochfrac,gate=passed,suffix_start_s=cut,suffix_epochs=len(suffix),suffix_label_fraction=suffixfraction,suffix_epoch_label_fraction=suffixepochfrac))
 packs[name]=(X,Y,np.concatenate(Zs),np.concatenate(Ts),np.concatenate(Obs),fr,np.concatenate(Q))
qualification=dict(groups='H+rawforward same full physics one train group; fresh E0 and E1 separate scene heldout groups; old V coverage-rejected unused',first_position_divergence_frame=first,suffix_start_s=cut,sequences=coverage,pass_gate=all(r['gate'] for r in coverage),source_sha256=sources)
(D/'fresh_qualification.json').write_text(json.dumps(qualification,indent=2)+'\n')
if not qualification['pass_gate']:
 (D/'fresh_result.json').write_text(json.dumps(dict(qualification_pass=False,fit=0,physics=0,reason='coverage_gate_failed'),indent=2)+'\n');raise SystemExit(0)
X,Y,_,_,_,fr,q=packs['H'];chosen=np.concatenate([idx[np.argsort(q[idx],kind='stable')][::8] for i in np.unique(fr) for idx in [np.flatnonzero((fr==i)&(Y>=0))]]);X=X[chosen];Y=Y[chosen];mean=X.mean(0);std=np.maximum(X.std(0),1e-6);A=np.column_stack([np.ones(len(X)),(X-mean)/std]);lam=.01
assert len(np.unique(Y))==2
with (D/'fit_started.json').open('x') as f:json.dump(dict(fit=1,rows=len(Y),validation_used=False,protocol_sha256=digest(D/'fresh_protocol.json'),code_sha256=digest(D/'calibrate_fresh.py'),qualification_sha256=digest(D/'fresh_qualification.json')),f)
def loss(w):
 t=A@w;v=np.mean(np.logaddexp(0,t)-Y*t)+lam*np.dot(w[1:],w[1:])/2;grad=A.T@(expit(t)-Y)/len(Y);grad[1:]+=lam*w[1:];return v,grad
opt=minimize(loss,np.zeros(A.shape[1]),jac=True,method='L-BFGS-B',options={'maxiter':150});np.savez(D/'model.npz',mean=mean,std=std,weights=opt.x);assert opt.success,opt.message
results=[]
for name,(X,Y,z,gt,obs,fr,q) in packs.items():
 prob=expit(np.column_stack([np.ones(len(X)),(X-mean)/std])@opt.x);keep=prob<.5
 for scope,m in [('whole',np.ones(len(X),bool))]:
  eligible=m&(Y>=0);retained=eligible&keep;e=np.abs(z[retained]-gt[retained]);pre=np.abs(z[eligible]-gt[eligible]);obret=(obs&m&keep).sum()/max(1,(obs&m).sum());allret=(m&keep).sum()/max(1,m.sum());epochrat=[keep[(fr==i)&m].mean() for i in np.unique(fr[m]) if ((fr==i)&m).sum()>=20];epass=np.mean(np.array(epochrat)>=.5) if epochrat else 0
  passed=len(e)>=1000 and np.quantile(e,.9)<=.15 and (e>.3).mean()<=.1 and obret>=.95 and allret>=.7 and epass>=.95
  results.append(dict(sequence=name,scope=scope,candidates=int(m.sum()),scored_before=len(pre),scored_after=len(e),before_p90_m=float(np.quantile(pre,.9)),after_p90_m=float(np.quantile(e,.9)),above030_fraction=float((e>.3).mean()),obstacle_retention=float(obret),all_candidate_retention=float(allret),epoch_retention_gate_fraction=float(epass),source_gate=bool(passed),positive_infinite_truth_candidates=int((m&np.isposinf(gt)).sum()),positive_infinite_truth_retained=int((m&np.isposinf(gt)&keep).sum())))
 np.savez_compressed(D/f'{name}_evaluation.npz',frame=fr,query=q,p_bad=prob,keep=keep,truth_valid=Y>=0,depth=z,gt_depth=gt)
r=dict(qualification_pass=True,fit=1,physics=0,model_iterations=int(opt.nit),results=results,heldout_gate=all(r['source_gate'] for r in results if r['sequence'] in ['E0','E1']),limits=['H training E2; E0/E1 frozen-source heldout; navigation increment unverified','No online consumer or navigation increment yet','Truth unavailable pixels excluded from error, counted in coverage; candidate retention covers all']);(D/'fresh_result.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2),flush=True)
