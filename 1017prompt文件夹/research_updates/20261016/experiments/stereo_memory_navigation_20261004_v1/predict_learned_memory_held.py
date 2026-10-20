"""Predict all fresh whole sequences from sealed features only, no GT reads."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np,torch
from memory_reliability_head import ReliabilityHead
H=Path(__file__).resolve().parent;F=H/'learned_memory_held_features';O=H/'learned_memory_held_predictions';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
weightseal=json.loads((H/'learned_memory_fit/fitted_weights_prevalidation_sha256.json').read_text());assert sha(H/'learned_memory_fit/Mmem1.pt')==weightseal['weight_sha256']
seal=json.loads((F/'feature_tables_prelabel_sha256.json').read_text())
for p,s in seal['sha256'].items():assert sha(p)==s,p
state=torch.load(H/'learned_memory_fit/Mmem1.pt',weights_only=True,map_location='cpu');model=ReliabilityHead(state['mean'].numpy(),state['scale'].numpy());model.load_state_dict(state);torch.set_num_threads(1)
(O/'protocol.json').write_text(json.dumps(dict(utc=now(),weight_sha256=weightseal['weight_sha256'],features_seal_sha256=sha(F/'feature_tables_prelabel_sha256.json'),source_sha256={str(p):sha(p) for p in [Path(__file__).resolve(),H/'memory_reliability_head.py',H/'learned_cell_memory.py']},threshold=.5,held_GT_labels_not_opened=True,model_imputed_only=True),indent=2)+'\n')
summary=[]
for case in ['memory_validation_far_run','memory_validation_near_run']:
 dest=O/case;dest.mkdir();n=0
 for p in sorted((F/case).glob('*.npz')):
  d=np.load(p);scores=model.predict(d['features'].astype(np.float32));np.savez_compressed(dest/p.name,scores=scores,cells=d['cells'],time_s=d['time_s'],last_query_times=d['last_query_times'],baseline_history_labels=d['baseline_history_labels'],origin=d['origin'],resolution=d['resolution'],model_outputs_are_observed=False);n+=len(scores)
 summary.append(dict(case=case,rows=n,frames=len(list(dest.glob('*.npz')))))
(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(O/'all_predictions_pretruth_sha256.json').write_text(json.dumps(dict(utc=now(),sha256={str(p):sha(p) for p in O.rglob('*') if p.is_file()},held_GT_labels_not_opened=True),indent=2)+'\n');print('ALL_FRESH_PREDICTIONS_SEALED_BEFORE_GT',summary)
