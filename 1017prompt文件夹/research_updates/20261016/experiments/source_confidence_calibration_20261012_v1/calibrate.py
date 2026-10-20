from pathlib import Path
import json,hashlib,time
import numpy as np,cv2
from scipy.optimize import minimize
from scipy.special import expit
D=Path(__file__).resolve().parent;W=D.parents[1]
H=W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1/candidate_e2_run'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def features(rgb,pred,packet):
 # Current input only; caller must not supply renderer truth.
 l=cv2.cvtColor(rgb['left_rgb'],cv2.COLOR_RGB2GRAY).astype(float);r=cv2.cvtColor(rgb['right_rgb'],cv2.COLOR_RGB2GRAY).astype(float)
 q=packet['query_pixel_id'].astype(int);p=packet['partner_pixel_id'].astype(int);x=q%640;y=q//640;xx=p%640;yy=p//640
 z=pred['depth_m'][y,x].astype(float);disparity=pred['disparity_px'];disparity=np.rot90(disparity,-1) if disparity.shape!=pred['depth_m'].shape else disparity;assert disparity.shape==pred['depth_m'].shape;disp=disparity[y,x].astype(float)
 def moment(im):
  m=cv2.boxFilter(im,-1,(7,7),normalize=True);sd=np.sqrt(np.maximum(cv2.boxFilter(im*im,-1,(7,7),normalize=True)-m*m,0));return m,sd
 ml,sl=moment(l);mr,sr=moment(r);valid=(x>=3)&(x<637)&(y>=3)&(y<477)&(xx>=3)&(xx<637)&(yy>=3)&(yy<477)
 ncc=np.zeros(len(q));sad=np.zeros(len(q))
 for a,b in [(a,b) for a in range(-3,4) for b in range(-3,4)]:
  lv=l[np.clip(y+a,0,479),np.clip(x+b,0,639)];rv=r[np.clip(yy+a,0,479),np.clip(xx+b,0,639)];ncc+=(lv-ml[y,x])*(rv-mr[yy,xx]);sad+=np.abs(lv-rv)
 ncc=ncc/(49*np.maximum(sl[y,x]*sr[yy,xx],1));sad/=49*255
 zz=np.nan_to_num(pred['depth_m'].astype(float),nan=0,posinf=0,neginf=0);dx=np.abs(zz[y,np.clip(x+1,0,639)]-zz[y,np.clip(x-1,0,639)]);dy=np.abs(zz[np.clip(y+1,0,479),x]-zz[np.clip(y-1,0,479),x]);world=packet['world']
 X=np.column_stack([z,disp/448,sl[y,x]/255,sr[yy,xx]/255,np.clip(ncc,-1,1),sad,dx,dy,world[:,2],packet['sigma_m'],np.abs(ml[y,x]-mr[yy,xx])/255,valid.astype(float)]);assert np.isfinite(X).all();return X,q
hs=json.loads((H/'states.json').read_text());vs=json.loads((V/'states.json').read_text());diff=[i for i,(h,v) in enumerate(zip(hs,vs)) if np.linalg.norm(np.array(h['position'])-v['position'])>1e-5];first=diff[0] if diff else 901;cut=first*.1+10
sources={};coverage=[];packs={}
for name,R in [('H',H),('V',V)]:
 rows=[];Xs=[];Ys=[];Zs=[];Ts=[];Obs=[];Frames=[];Q=[]
 predpaths=sorted((R/'source_predictions').glob('*.npz'));assert len(predpaths)==181
 states=json.loads((R/'states.json').read_text());term=json.loads((R/'termination.json').read_text());assert len(states)==901 and term['termination']=='timeout90';assert abs(states[-1]['time_s']-90)<1e-4
 for pp in predpaths:
  i=int(pp.stem.split('_')[1]);paths=[pp,R/'source_inputs'/pp.name,R/'sensor_packets'/pp.name.replace('frame_','source_'),R/'frames'/pp.name]
  pred=dict(np.load(paths[0]));rgb=dict(np.load(paths[1]));packet=dict(np.load(paths[2]));assert abs(float(pred['time_s'])-float(rgb['time_s']))<1e-7;assert abs(float(pred['time_s'])-float(packet['time_s']))<1e-7
  X,q=features(rgb,pred,packet);f=np.load(paths[3]);assert abs(float(f['time_s'])-float(pred['time_s']))<1e-7;y=q//640;x=q%640;gt=f['left_depth_m'][y,x];ok=np.isfinite(gt)&(gt>0);z=pred['depth_m'][y,x];label=np.abs(z-gt)>.15
  T=f['left_camera_usd_world'];rays=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1);gtworld=T[:3,3]+(rays*np.where(ok,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;obstacle=ok&(gtworld[:,2]>.05)&(gtworld[:,2]<=1.5)
  rows.append(dict(frame=i,t=float(pred['time_s']),candidates=len(q),labels=int(ok.sum()),bad=int((label&ok).sum())))
  Xs.append(X);Ys.append(np.where(ok,label,-1));Zs.append(z);Ts.append(gt);Obs.append(obstacle);Frames.append(np.full(len(q),i));Q.append(q)
  for p in paths:sources[str(p)]=digest(p)
  if i%100==0:print(name,i,flush=True)
 X=np.concatenate(Xs);Y=np.concatenate(Ys);fr=np.concatenate(Frames);eligible=[r for r in rows if r['candidates']>=20];fraction=sum(r['labels'] for r in rows)/max(1,sum(r['candidates'] for r in rows));epochfrac=sum(r['labels']>=20 for r in eligible)/max(1,len(eligible));suffix=[r for r in rows if r['t']>cut]
 suffixfraction=sum(r['labels'] for r in suffix)/max(1,sum(r['candidates'] for r in suffix));suffixeligible=[r for r in suffix if r['candidates']>=20];suffixepochfrac=sum(r['labels']>=20 for r in suffixeligible)/max(1,len(suffixeligible));passed=(fraction>=.70 and epochfrac>=.95) if name=='H' else (suffixfraction>=.70 and suffixepochfrac>=.95 and 90-cut>=30 and len(suffix)>=50);coverage.append(dict(sequence=name,rows=rows,label_fraction=fraction,eligible_epoch_label_fraction=epochfrac,gate=passed,suffix_start_s=cut,suffix_epochs=len(suffix),suffix_label_fraction=suffixfraction,suffix_epoch_label_fraction=suffixepochfrac))
 packs[name]=(X,Y,np.concatenate(Zs),np.concatenate(Ts),np.concatenate(Obs),fr,np.concatenate(Q))
qualification=dict(groups='H+rawforward same full physics; V distinct post-divergence suffix only, same E2 development not independent scenario',first_position_divergence_frame=first,suffix_start_s=cut,sequences=coverage,pass_gate=all(r['gate'] for r in coverage),source_sha256=sources)
(D/'qualification.json').write_text(json.dumps(qualification,indent=2)+'\n')
if not qualification['pass_gate']:
 (D/'result.json').write_text(json.dumps(dict(qualification_pass=False,fit=0,physics=0,reason='coverage_gate_failed'),indent=2)+'\n');raise SystemExit(0)
X,Y,_,_,_,fr,q=packs['H'];chosen=np.concatenate([idx[np.argsort(q[idx],kind='stable')][::8] for i in np.unique(fr) for idx in [np.flatnonzero((fr==i)&(Y>=0))]]);X=X[chosen];Y=Y[chosen];mean=X.mean(0);std=np.maximum(X.std(0),1e-6);A=np.column_stack([np.ones(len(X)),(X-mean)/std]);lam=.01
assert len(np.unique(Y))==2
(D/'fit_started.json').write_text(json.dumps(dict(fit=1,rows=len(Y),validation_used=False,protocol_sha256=digest(D/'protocol.json')))+'\n')
def loss(w):
 t=A@w;v=np.mean(np.logaddexp(0,t)-Y*t)+lam*np.dot(w[1:],w[1:])/2;grad=A.T@(expit(t)-Y)/len(Y);grad[1:]+=lam*w[1:];return v,grad
opt=minimize(loss,np.zeros(A.shape[1]),jac=True,method='L-BFGS-B',options={'maxiter':150});np.savez(D/'model.npz',mean=mean,std=std,weights=opt.x);assert opt.success,opt.message
results=[]
for name,(X,Y,z,gt,obs,fr,q) in packs.items():
 prob=expit(np.column_stack([np.ones(len(X)),(X-mean)/std])@opt.x);keep=prob<.5
 for scope,m in [('whole',np.ones(len(X),bool)),('unshared_suffix',fr*.1>cut)]:
  eligible=m&(Y>=0);retained=eligible&keep;e=np.abs(z[retained]-gt[retained]);pre=np.abs(z[eligible]-gt[eligible]);obret=(obs&m&keep).sum()/max(1,(obs&m).sum());allret=(m&keep).sum()/max(1,m.sum());epochrat=[keep[(fr==i)&m].mean() for i in np.unique(fr[m]) if ((fr==i)&m).sum()>=20];epass=np.mean(np.array(epochrat)>=.5) if epochrat else 0
  passed=len(e)>=1000 and np.quantile(e,.9)<=.15 and (e>.3).mean()<=.1 and obret>=.95 and allret>=.7 and epass>=.95
  results.append(dict(sequence=name,scope=scope,candidates=int(m.sum()),scored_before=len(pre),scored_after=len(e),before_p90_m=float(np.quantile(pre,.9)),after_p90_m=float(np.quantile(e,.9)),above030_fraction=float((e>.3).mean()),obstacle_retention=float(obret),all_candidate_retention=float(allret),epoch_retention_gate_fraction=float(epass),source_gate=bool(passed)))
 np.savez_compressed(D/f'{name}_evaluation.npz',frame=fr,query=q,p_bad=prob,keep=keep,truth_valid=Y>=0,depth=z,gt_depth=gt)
r=dict(qualification_pass=True,fit=1,physics=0,model_iterations=int(opt.nit),results=results,heldout_gate=next(r['source_gate'] for r in results if r['sequence']=='V' and r['scope']=='unshared_suffix'),limits=['H/V both E2 development; fresh E0/E1 required','No online consumer or navigation increment yet','Truth unavailable pixels excluded from error, counted in coverage; candidate retention covers all']);(D/'result.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2),flush=True)
