from pathlib import Path
import json,numpy as np,time
from learned_memory_risk_dataset_v1 import feature_only_sequence,sha
from learned_memory_risk_model_v1 import predict_features
from learned_memory_risk_identity_v1 import verify_prefit_identity
H=Path(__file__).resolve().parent
if __name__=='__main__':
 prefit_sha=verify_prefit_identity(H)
 design=json.loads((H/'learned_memory_risk_qualifier_design_v1.json').read_text());model=json.loads((H/'learned_memory_risk_model_v1.json').read_text());fitseal=json.loads((H/'learned_memory_risk_model_fit_seal_v1.json').read_text());assert sha(H/'learned_memory_risk_model_v1.json')==fitseal['model_sha256']
 assert fitseal['prefit_identity_sha256']==prefit_sha
 for source,digest in fitseal['source_sha256'].items():assert sha(source)==digest,source
 outputs={}
 for name in design['fresh_whole_sequence_holdout']:
  out=H/(name+'_risk_predictions_v1.npz');assert not out.exists();assert sha(H/name/'artifact_manifest_post_close.json')==fitseal['heldout_input_seals'][name]
  X,index,inputs=feature_only_sequence(H/name);begin=time.monotonic();p,keep=predict_features(model,X);wall=time.monotonic()-begin
  np.savez_compressed(out,probability=p,keep=keep,frame=np.array([r['frame'] for r in index]),time_s=np.array([r['time_s'] for r in index]),query_pixel_id=np.array([r['query_pixel_id'] for r in index]))
  meta=out.with_suffix('.json');meta.write_text(json.dumps(dict(input_sha256=inputs,index=index,model_sha256=fitseal['model_sha256'],prediction_points=len(index),prediction_wall_s=wall,heldout_teacher_read=False),indent=2)+'\n');outputs[str(out)]=sha(out);outputs[str(meta)]=sha(meta)
 seal=H/'learned_memory_risk_holdout_predictions_seal_v1.json';assert not seal.exists();seal.write_text(json.dumps(dict(prefit_identity_sha256=prefit_sha,outputs_sha256=outputs,model_sha256=fitseal['model_sha256'],two_complete_sequence_predictions_sealed_before_teacher=True,not_online_validation=True),indent=2)+'\n');print('TWO_FEATURE_ONLY_HOLDOUT_PREDICTIONS_SEALED')
