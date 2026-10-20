"""Twenty archived outputs with declared GT-pose upperbound; no NN call/labels."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from paired_texture_surface import PairedTextureSurfaceMemory
from static_base_cached_self_mask_v2 import StaticBaseCachedSelfMask
from masked_source_pair import paired_sources
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;O=H/'frozen_partial_original_scene_transfer_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
c=json.loads((H/'frozen_partial_original_scene_transfer_protocol_v1.json').read_text())
for group in ('source_sha256','input_sha256'):
 for f,s in c[group].items():assert sha(f)==s,f
old=json.loads((H/'transaction_development_manifest.json').read_text());assert len(old['samples'])==20
oldmask=np.load(H.parent/'visual_information_new_rig_20261004_v2/base_link_masks.npz');newmask=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
for eye in ('left','right'):np.testing.assert_array_equal(oldmask[eye],newmask[eye])
oldproto=json.loads((H/'transaction_development_protocol.json').read_text());assert oldproto['source_sha256'][str(H/'fast_stereo.py')]==sha(H/'fast_stereo.py')
O.mkdir(exist_ok=False);cad=StaticBaseCachedSelfMask();memory={};rows=[]
for r in old['samples']:
 seq=r['sequence'];name=Path(r['rgb_file']).name;i=int(Path(name).stem.split('_')[-1]);rgb=np.load(r['rgb_file']);p=np.load(H/'predictions_transaction_development/fast'/seq/name)
 # GT-file access is intentionally restricted to declared pose/time input.
 with np.load(r['truth_file']) as z:pose={k:z[k].copy() for k in ('left_camera_usd_world','right_camera_usd_world','root_position_world_m')};t=float(z['simulation_time_s'])
 assert t==float(rgb['simulation_time_s']);history=rgb['proprio_history'].copy();history[:,:9]=0.
 _,raw,_=paired_sources(dict(depth_m=p['depth_m'],confidence=p['confidence'],disparity_native_px=restore_native(p['disparity_px']),frame=i,time_s=t),dict(**pose,time_s=t),history,cad,t)
 m=memory.setdefault(seq,PairedTextureSurfaceMemory());hashes={r['rgb_file']:r['rgb_sha256'],r['truth_file']:r['truth_sha256']};points=m.update(raw,rgb['left_rgb'],rgb['right_rgb'],t,hashes,restore_native(p['disparity_px']))
 dest=O/seq;dest.mkdir(exist_ok=True);np.savez_compressed(dest/name,**{k:v for k,v in points.items() if k!='input_hashes'})
 rows.append(dict(sequence=seq,frame=i,time_s=t,points=len(points['world']),rgb_file=r['rgb_file'],truth_file=r['truth_file'],output=str(dest/name),input_sha256=hashes,privileged_true_pose_upperbound=True,depth_labels_not_read_for_selection=True))
(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),code_sha256=sha(__file__),protocol_sha256=sha(H/'frozen_partial_original_scene_transfer_protocol_v1.json'),outputs={str(p.resolve()):sha(p) for p in O.rglob('*') if p.is_file()},no_NN_fit_or_forward_or_physics=True,not_fresh_or_online=True),indent=2)+'\n');print(json.dumps({'saved':len(rows),'selected_per_sequence':{s:sum(r['points'] for r in rows if r['sequence']==s) for s in memory},'source_outputs_sealed_before_error_labels':True},indent=2))
