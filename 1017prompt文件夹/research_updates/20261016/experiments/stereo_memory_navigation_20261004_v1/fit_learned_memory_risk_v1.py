from pathlib import Path
import json,hashlib,datetime,numpy as np
from sklearn.linear_model import LogisticRegression
from learned_memory_risk_features_v1 import FEATURE_NAMES
from learned_memory_risk_dataset_v1 import feature_only_sequence,offline_teacher_labels,verify_sequence,sha
from learned_memory_risk_identity_v1 import verify_prefit_identity
H=Path(__file__).resolve().parent
if __name__=='__main__':
 prefit_sha=verify_prefit_identity(H)
 design=json.loads((H/'learned_memory_risk_qualifier_design_v1.json').read_text());reservation=json.loads((H/'learned_memory_risk_holdout_reservation_v1.json').read_text());assert reservation['design_sha256']==sha(H/'learned_memory_risk_qualifier_design_v1.json') and reservation['runs_absent_at_freeze']
 for name in design['fresh_whole_sequence_holdout']:verify_sequence(H/name)
 modelpath=H/'learned_memory_risk_model_v1.json';assert not modelpath.exists()
 xs=[];ys=[];provenance={};counts={}
 for name in design['training_whole_sequences']:
  X,index,feature_inputs=feature_only_sequence(H/name);y,height,teacher_inputs=offline_teacher_labels(H/name,index);valid=y>=0;xs.append(X[valid]);ys.append(y[valid]);counts[name]=dict(all_points=len(y),floor=int((y==0).sum()),hazard=int((y==1).sum()),unlabelled=int((y<0).sum()));provenance[name]=dict(feature_inputs=feature_inputs,training_only_teacher_inputs=teacher_inputs,whole_sequence_seal=sha(H/name/'artifact_manifest_post_close.json'))
 X=np.vstack(xs);y=np.concatenate(ys);assert len(np.unique(y))==2
 mean=X.mean(axis=0);scale=X.std(axis=0);scale=np.where(scale>1e-12,scale,1.);normalized=(X-mean)/scale
 learner=LogisticRegression(C=1.,penalty='l2',class_weight='balanced',solver='lbfgs',max_iter=200,random_state=0);learner.fit(normalized,y)
 model=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),candidate='one balanced logistic current/past hazard-memory insertion qualifier',feature_names=FEATURE_NAMES,mean=mean.tolist(),scale=scale.tolist(),coef=learner.coef_[0].tolist(),intercept=float(learner.intercept_[0]),threshold=.5,iterations=learner.n_iter_.tolist(),converged=bool(np.max(learner.n_iter_)<200),training_counts=counts,training_provenance=provenance,design_sha256=sha(H/'learned_memory_risk_qualifier_design_v1.json'),heldout_teacher_not_read_by_fit=True,not_dynamics_world_model=True,unknown_not_free=True,whole_mission_complete=False)
 modelpath.write_text(json.dumps(model,indent=2)+'\n')
 (H/'learned_memory_risk_model_fit_seal_v1.json').write_text(json.dumps(dict(prefit_identity_sha256=prefit_sha,model_sha256=sha(modelpath),source_sha256={str(H/n):sha(H/n) for n in ['learned_memory_risk_features_v1.py','learned_memory_risk_dataset_v1.py','learned_memory_risk_model_v1.py','fit_learned_memory_risk_v1.py']},heldout_input_seals={n:sha(H/n/'artifact_manifest_post_close.json') for n in design['fresh_whole_sequence_holdout']},fit_count=1,search=False,heldout_teacher_not_inspected=True),indent=2)+'\n');print(json.dumps(dict(counts=counts,iterations=model['iterations'],converged=model['converged'])))
