"""Seal new whole sequence BEFOREfit; build old-only sanitized fit inputs."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent;W=H.parent.parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
status=json.loads((H/'dense_adapter_heldout_status.json').read_text());assert status['status']=='finished' and status['exit_code']==0
out=H/'dense_adapter_heldout_run';identity=json.loads((out/'identity.json').read_text());assert identity['complete'] and identity['captures']==321 and identity['saved_frames']==65 and identity['internal_reset'] is False
phys=np.load(out/'physics.npz');assert len(phys['simulation_time_s'])==1601
clocks=json.loads((out/'camera_clocks.json').read_text());assert len(clocks)==321 and all(abs(r['raw_time_s']-v)<=1e-5 for r in clocks for v in r['eyes'].values())
original=json.loads((out/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for f,d in original.items():assert sha(f)==d,f
folder=H/'dense_adapter_heldout_rgb';folder.mkdir(exist_ok=False);inputs={}
assert {p.name for p in (out/'frames').glob('*.npz')}=={f'frame_{i:03d}.npz' for i in range(0,321,5)}
for frame in range(0,321,5):
 file=out/'frames'/f'frame_{frame:03d}.npz';raw=np.load(file);dest=folder/file.name
 # ONLY RGB/time decoded here; true depth remains opaque until after full predictions.
 np.savez_compressed(dest,left_rgb=raw['left_rgb'],right_rgb=raw['right_rgb'],time_s=raw['time_s']);inputs[str(dest)]=sha(dest)
seal=dict(utc=now(),artifact_sha256=original,sanitized_rgb_sha256=inputs,whole_sequence=True,initial_yaw_offset_deg=-20,labels_decoded_for_design_or_fit=False,frames=65,domain='new physical sequence/different yaw; existing E2scene, not newenvironment')
(H/'dense_adapter_heldout_seal.json').write_text(json.dumps(seal,indent=2)+'\n')
fitdir=H/'dense_adapter_fit_inputs';fitdir.mkdir(exist_ok=False);examples=[];oldinputs={}
seqs=[('e1',H/'oracle_rgb_shadow_run',list(range(0,281,28))+[307]),('hard_0_e1_0004',H/'capture_transaction_development/hard_0_e1_0004',[0,10,20,23])]
for seq,root,frames in seqs:
 d=fitdir/seq;d.mkdir()
 for i in frames:
  rgbfile=root/'online_inputs'/f'frame_{i:03d}.npz';truthfile=root/'offline_truth'/f'frame_{i:03d}.npz';rgb=np.load(rgbfile);label=np.load(truthfile);a=d/f'rgb_{i:03d}.npz';b=d/f'label_{i:03d}.npz'
  np.savez_compressed(a,left_rgb=rgb['left_rgb'],right_rgb=rgb['right_rgb']);np.savez_compressed(b,left_depth_m=label['left_depth_m'],right_depth_m=label['right_depth_m'])
  for f in (rgbfile,truthfile,a,b):oldinputs[str(f)]=sha(f)
  for eye in ('left','right'):examples.append(dict(sequence=seq,frame=i,eye=eye,rgb=str(a),labels=str(b)))
assert len(examples)==32
sourcefiles=[H/f for f in ['dense_adapter.py','test_dense_adapter.py','dense_adapter_metrics.py','test_dense_adapter_metrics.py','fit_dense_adapter.py','predict_dense_adapter.py','assess_dense_adapter.py','seal_dense_adapter_fit.py','fast_stereo.py','checkpoint_loader.py','stereo_geometry.py','audit_stereo.py','run_dense_adapter_heldout.py','run_dense_adapter_heldout_stage.py','run_dense_adapter_training_stage.py']]+list((H/'third_party/fast_foundation_stereo').rglob('*.py'))
sourcefiles += [W/'docs/superpowers/specs/2026-10-06-dense-stereo-domain-adapter-design.md',W/'docs/superpowers/plans/2026-10-06-dense-stereo-domain-adapter.md',W/'docs/superpowers/specs/2026-10-06-dense-adapter-execution-addendum.md']
mask=H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz';weight=H/'third_party/fast_foundation_stereo/weights/23-36-37/model_best_bp2_serialize.pth'
inputsha={**oldinputs,**inputs,str(mask):sha(mask),str(weight):sha(weight)}
p=dict(utc=now(),examples=examples,mask=str(mask),heldout_seal_sha256=sha(H/'dense_adapter_heldout_seal.json'),source_sha256={str(f):sha(f) for f in sourcefiles},input_sha256=inputsha,steps=300,lr=1e-5,weight_decay=1e-4,gradclip=1,seed=0,scaler_init_scale=1,scaler_growth_interval=1000,trainable_prefixes=['update_block.','spx_2_gru.','spx_gru.'],eager_execution='TORCH_COMPILE_DISABLE=1',whole_sequence_fresh_pre_fit=True,whole_research_complete=False)
(H/'dense_adapter_fit_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print('SEALED',len(original),len(inputsha),len(sourcefiles),len(examples))
