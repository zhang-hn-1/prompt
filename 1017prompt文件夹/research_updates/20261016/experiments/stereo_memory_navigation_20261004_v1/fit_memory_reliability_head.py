"""Fit exactly one candidate after complete fresh-sequence sealing, fit labels only."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np,torch
from memory_reliability_head import ReliabilityHead
H=Path(__file__).resolve().parent;F=H/'learned_memory_fit_features_v2';O=H/'learned_memory_fit';O.mkdir(exist_ok=False);now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();status=json.loads((H/'memory_validation_collection_status.json').read_text());assert status['status']=='finished_both_validation_sequences_sealed'
held={}
for kind in ['far','near']:
 o=H/f'memory_validation_{kind}_run';p=o/'artifact_manifest_post_close.json';assert p.exists();held[str(p)]=sha(p);identity=json.loads((o/'identity.json').read_text());assert identity['policy_calls']==180 and identity['captures']==181 and identity['heldout_for_learning']
features_seal=json.loads((F/'feature_tables_prelabel_sha256.json').read_text())
for p,s in features_seal['sha256'].items():assert sha(p)==s,p
cases=json.loads((F/'protocol.json').read_text())['fit_cases'];assert len(cases)==5 and all('validation' not in case for case in cases);sources=[Path(__file__).resolve(),H/'memory_reliability_head.py',H/'learned_cell_memory.py',H/'packet_scalars.py',H.parents[1]/'docs/superpowers/plans/2026-10-07-learned-memory-reliability.md'];label_paths=[H/case/'offline_occupancy_label.npz' for case in cases]
protocol=dict(utc=now(),source_sha256={str(p):sha(p) for p in sources},fit_feature_seal_sha256=sha(F/'feature_tables_prelabel_sha256.json'),held_manifest_identity_only=held,heldout_GT_labels_and_scores_not_opened=True,heldout_features_not_opened=True,fit_label_sha256={str(p):sha(p) for p in label_paths},single_architecture='21-16tanh-1 sigmoid score, deterministic causal35s memory',seed=7,epochs=10,batch=256,optimizer='Adam lr=.003',loss='classbalancedBCE',threshold=.5,whole_sequence_split=True,static_2D_proxy_not_depth_or_safety_certificate=True)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');X=[];Y=[];index=[]
# Fit GT proxy values read only now, after feature-table sealing; no heldout labels.
for case in cases:
 occ=np.load(H/case/'offline_occupancy_label.npz');n=0;pos=0
 for p in sorted((F/case).glob('*.npz')):
  d=np.load(p);xy=d['origin']+d['cells']*float(d['resolution']);ij=np.rint((xy-occ['origin'])/float(occ['resolution'])).astype(int);valid=(ij>=0).all(1)&(ij<occ['labels'].shape).all(1);y=occ['labels'][ij[valid,0],ij[valid,1]]>0;x=d['features'][valid];X.append(x);Y.append(y.astype(np.float32));n+=len(x);pos+=int(y.sum())
 index.append(dict(case=case,fit_rows=n,positive_static_occupied_cell_rows=pos))
X=np.concatenate(X).astype(np.float32);Y=np.concatenate(Y);assert X.shape[1]==21 and np.isfinite(X).all() and 0<Y.sum()<len(Y);np.savez_compressed(O/'fit_dataset.npz',features=X,labels=Y);(O/'fit_index.json').write_text(json.dumps(index,indent=2)+'\n')
torch.set_num_threads(1);torch.manual_seed(7);torch.use_deterministic_algorithms(True);mean=X.mean(0);scale=X.std(0);scale[scale<1e-6]=1.;model=ReliabilityHead(mean,scale);optimizer=torch.optim.Adam(model.parameters(),lr=.003);lossfn=torch.nn.BCEWithLogitsLoss(pos_weight=torch.tensor(float((len(Y)-Y.sum())/Y.sum())));tx=torch.tensor(X);ty=torch.tensor(Y);generator=torch.Generator().manual_seed(7);history=[]
for epoch in range(10):
 model.train();order=torch.randperm(len(X),generator=generator);losses=[]
 for ids in order.split(256):
  optimizer.zero_grad();loss=lossfn(model(tx[ids]),ty[ids]);loss.backward();optimizer.step();losses.append(float(loss.detach()))
 history.append(dict(epoch=epoch+1,mean_batch_BCE=float(np.mean(losses))));print('FIT_EPOCH',history[-1],flush=True)
model.eval();torch.save(model.state_dict(),O/'Mmem1.pt');fit_prob=model.predict(X);np.savez_compressed(O/'fit_predictions.npz',scores=fit_prob,labels=Y);(O/'training_log.json').write_text(json.dumps(history,indent=2)+'\n');(O/'fitted_weights_prevalidation_sha256.json').write_text(json.dumps(dict(utc=now(),weight_sha256=sha(O/'Mmem1.pt'),protocol_sha256=sha(O/'protocol.json'),fit_dataset_sha256=sha(O/'fit_dataset.npz'),held_manifests=held,heldout_labels_features_scores_not_opened=True,one_fit_only=True),indent=2)+'\n');print('ONE_MMEM1_FROZEN_BEFORE_ANY_HELD_PREDICTION_OR_LABEL',len(X),int(Y.sum()))
