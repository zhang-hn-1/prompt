"""Single fit after whole fresh acquisition seals. Offline causal forecast, not online shadow."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
from action_video_forecast_v1 import causal_features,fit_once,predict
H=Path(__file__).resolve().parent;cfg=json.loads((H/'action_conditioned_dual_video_forecast_protocol_v1.json').read_text());O=H/'action_video_forecast_fit_validation_v1'
LK=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
y,x=np.mgrid[12:480:24,12:640:24];Q=np.column_stack([x.ravel(),y.ravel()]).astype(np.float32);P=Q.reshape(-1,1,2)
MASK=dict(np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz'))
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify_seal(D):
 s=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==h for p,h in s['artifact_sha256'].items());return sha(D/'artifact_manifest_post_close.json')
def eligible(xy,eye):
 xy=np.asarray(xy);finite=np.isfinite(xy).all(1);inside=finite&(xy[:,0]>=0)&(xy[:,0]<640)&(xy[:,1]>=0)&(xy[:,1]<480);out=np.zeros(len(xy),bool);v=xy[inside].astype(int);out[inside]=~MASK[eye][v[:,1],v[:,0]];return out

def past_flow(prev,cur,eye):
 back,ok,err=cv2.calcOpticalFlowPyrLK(cur,prev,P,None,**LK);forth,fok,_=cv2.calcOpticalFlowPyrLK(prev,cur,back,None,**LK)
 v=ok.ravel().astype(bool)&fok.ravel().astype(bool)&eligible(back[:,0],eye)&eligible(Q,eye)&np.isfinite(forth[:,0]).all(1)&(np.linalg.norm(forth[:,0]-Q,axis=1)<=1)&(err.ravel()<=20)
 return Q-back[:,0],v

def build_inputs(name,model=None):
 D=H/name;req=np.load(D/'requests.npz')['requests'];states=json.loads((D/'states.json').read_text());packets=json.loads((D/'native_imu_packets.json').read_text());lookup={round(float(p['time_s']),6):p for p in packets if p is not None};joints=np.load(D/'physics.npz')['joint_position_rad'];cap=cv2.VideoCapture(str(D/'stereo_view.mp4'));ok,prev=cap.read();assert ok and prev.shape[:2]==(480,1280);indices=[];features=[];baselines=[];movings=[];predictions=[];times=[]
 for i in range(1,len(req)):
  ok,cur=cap.read();assert ok and cur.shape[:2]==(480,1280);t=float(states[i]['time_s']);imu=lookup[round(t,6)];assert abs(float(imu['time_s'])-t)<1e-6
  sensor=np.r_[imu['gyro_body_rad_s'],imu['linear_acceleration_body_m_s2'],imu['main_encoder_rad_s'],joints[i*5],req[i,:2],req[i-1,:2]];moving=abs(float(imu['main_encoder_rad_s']))>.05 or np.linalg.norm(imu['gyro_body_rad_s'])>.05;fx=[];bx=[];pr=[]
  for en,eye in enumerate(('left','right')):
   a=cv2.cvtColor(prev[:,en*640:(en+1)*640],cv2.COLOR_BGR2GRAY);b=cv2.cvtColor(cur[:,en*640:(en+1)*640],cv2.COLOR_BGR2GRAY);flow,valid=past_flow(a,b,eye);f,base=causal_features(sensor,Q,flow,valid,en,t,t);fx.append(f);bx.append(base)
   if model is not None:pr.append(predict(model,f,base))
  features.append(fx);baselines.append(bx);movings.append(moving);indices.append(i);times.append(t)
  if model is not None:predictions.append(pr)
  # No next image read until this current-only feature/prediction has been computed.
  prev=cur
 cap.release();f=O/(name+'_causal_inputs.npz');np.savez_compressed(f,features=np.asarray(features),baseline=np.asarray(baselines),moving=np.asarray(movings),indices=np.asarray(indices),times_s=np.asarray(times),query_xy=Q)
 result=dict(input_sha256=sha(f),offline_video_reader_current_past_only=True,no_future_feedback_or_pose=True,not_online_interface_validation=True)
 if model is not None:
  p=O/(name+'_predictions_before_future_teacher.npz');np.savez_compressed(p,prediction=np.asarray(predictions));result['prediction_sha256']=sha(p);result['model_sha256']=sha(O/'model.npz')
 (O/(name+'_prediction_or_input_seal.json')).write_text(json.dumps(dict(utc=now(),**result),indent=2)+'\n');return dict(np.load(f)),result

def filter_teacher(flow,valid):
 out=valid.copy()
 for en,eye in enumerate(('left','right')):
  for k in range(len(flow)):out[k,en]&=eligible(Q,eye)&eligible(Q+flow[k,en],eye)
 return out

def historical_teacher(name):
 L=H/'action_conditioned_video_pair_feasibility_v1';R=H/'right_action_video_pair_supplement_v1';l=np.load(L/(name+'_future_flow_labels.npz'));r=np.load(R/(name+'_future_right_flow_labels.npz'));flow=np.stack([l['flow'],r['flow']],axis=1);valid=np.stack([l['usable_LK'],r['usable_LK']],axis=1);return flow,filter_teacher(flow,valid)

def fresh_teacher(name,indices):
 cap=cv2.VideoCapture(str(H/name/'stereo_view.mp4'));images=[]
 while True:
  ok,im=cap.read()
  if not ok:break
  images.append(im)
 cap.release();flow=[];valid=[]
 for i in indices:
  fy=[];vy=[]
  for en,eye in enumerate(('left','right')):
   a=cv2.cvtColor(images[i][:,en*640:(en+1)*640],cv2.COLOR_BGR2GRAY);b=cv2.cvtColor(images[i+1][:,en*640:(en+1)*640],cv2.COLOR_BGR2GRAY);f,ok,err=cv2.calcOpticalFlowPyrLK(a,b,P,None,**LK);back,bok,_=cv2.calcOpticalFlowPyrLK(b,a,f,None,**LK);v=ok.ravel().astype(bool)&bok.ravel().astype(bool)&np.isfinite(f[:,0]).all(1)&(np.linalg.norm(back[:,0]-Q,axis=1)<=1)&(err.ravel()<=20);fy.append(f[:,0]-Q);vy.append(v)
  flow.append(fy);valid.append(vy)
 flow=np.asarray(flow);return flow,filter_teacher(flow,np.asarray(valid))

def main():
 identity=json.loads((H/'action_video_forecast_fit_inputs_identity_v1.json').read_text());assert all(sha(p)==h for p,h in identity['input_sha256'].items());label_seals={name:verify_seal(H/name) for name in ('action_conditioned_video_pair_feasibility_v1','right_action_video_pair_supplement_v1')}
 status=json.loads((H/'action_video_fresh_status_v1.json').read_text());assert status['status']=='finished' and all(s['normal_close_confirmed'] for s in status['sequences']);assert not O.exists();sealed={n:verify_seal(H/n) for n in cfg['fit_whole_sequences']+cfg['fresh_whole_sequences']};O.mkdir()
 (O/'fresh_whole_sealed_before_fit.json').write_text(json.dumps(dict(utc=now(),seals=sealed,protocol_sha256=sha(H/'action_conditioned_dual_video_forecast_protocol_v1.json'),source_sha256=sha(__file__),module_sha256=sha(H/'action_video_forecast_v1.py'),fit_label_and_mask_identity=identity,label_directory_seals=label_seals),indent=2)+'\n')
 X=[];Y=[];B=[];W=[];trainrows=[]
 for n in cfg['fit_whole_sequences']:
  a,seal=build_inputs(n);y,v=historical_teacher(n);assert y.shape==a['baseline'].shape
  count=v.sum(axis=(1,2));w=np.broadcast_to(np.divide(1.,count,out=np.zeros(len(count)),where=count>0)[:,None,None],v.shape)
  X.append(a['features'][v]);Y.append(y[v]);B.append(a['baseline'][v]);W.append(w[v]);trainrows.append(dict(sequence=n,usable=int(v.sum()),whole_historical_fit_only=True))
 model=fit_once(np.concatenate(X),np.concatenate(Y),np.concatenate(B),np.concatenate(W));np.savez_compressed(O/'model.npz',**model);rows=[];allpass=True
 # Seal predictions for BOTH whole fresh sequences before opening their future teacher labels.
 inputs={};predseals={}
 for n in cfg['fresh_whole_sequences']:inputs[n],predseals[n]=build_inputs(n,model)
 (O/'both_fresh_predictions_sealed_before_teacher.json').write_text(json.dumps(dict(utc=now(),seals=predseals),indent=2)+'\n')
 for n in cfg['fresh_whole_sequences']:
  a=inputs[n];pred=np.load(O/(n+'_predictions_before_future_teacher.npz'))['prediction'];y,v=fresh_teacher(n,a['indices']);np.savez_compressed(O/(n+'_future_teacher_after_predictions.npz'),flow=y,valid=v)
  assert sha(O/(n+'_predictions_before_future_teacher.npz'))==predseals[n]['prediction_sha256']
  for en,eye in enumerate(('left','right')):
   usable=v[:,en];moving=usable&a['moving'][:,None];err=np.linalg.norm(pred[:,en]-y[:,en],axis=2);berr=np.linalg.norm(a['baseline'][:,en]-y[:,en],axis=2);m=float(err[moving].mean()) if moving.any() else None;b=float(berr[moving].mean()) if moving.any() else None;ratio=m/b if b and b>0 else None;p90=float(np.percentile(err[moving],90)) if moving.any() else None;frames=int(np.sum(moving.any(axis=1)));passed=bool(moving.sum()>=100 and frames>=10 and ratio is not None and ratio<=.85 and p90<=5);allpass&=passed
   row=dict(sequence=n,eye=eye,usable_all_queries=int(usable.sum()),usable_moving_queries=int(moving.sum()),usable_moving_frames=frames,moving_mean_EPE_px=m,baseline_moving_mean_EPE_px=b,moving_mean_ratio=ratio,moving_p90_EPE_px=p90,all_mean_EPE_px=float(err[usable].mean()) if usable.any() else None,passed=passed);rows.append(row);print(row,flush=True)
 (O/'assessment.json').write_text(json.dumps(dict(utc=now(),fit=trainrows,fresh=rows,accepted_forecast_layer_only=allpass,decision='forecast pass eligible for bounded readonly online shadow' if allpass else 'sole predictor rejected; no refit/alternative/network or gate change',offline_causal_prediction_not_online=True,teacher_approximate_2D_not_3D_safety=True,no_actor_change=True,formal_navigation_admission=False,whole_mission_complete=False),indent=2)+'\n');(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):sha(p) for p in O.rglob('*') if p.is_file()}),indent=2)+'\n')
if __name__=='__main__':main()
