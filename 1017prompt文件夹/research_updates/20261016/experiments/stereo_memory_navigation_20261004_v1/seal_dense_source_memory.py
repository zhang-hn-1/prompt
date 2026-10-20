from pathlib import Path
import json,datetime,hashlib,numpy as np
H=Path(__file__).resolve().parent;W=H.parent.parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();folder=H/'dense_source_memory_knownposes';folder.mkdir(exist_ok=False);inputs={}
for i in range(0,321,5):
 rawfile=H/'dense_adapter_heldout_run/frames'/f'frame_{i:03d}.npz';raw=np.load(rawfile);dest=folder/rawfile.name
 # only these permitted currentknownstate fields decoded; GTdepth notinput.
 np.savez_compressed(dest,**{k:raw[k] for k in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']});inputs[str(dest)]=sha(dest)
first=np.load(folder/'frame_000.npz');origin=(first['root_position_world_m'][:2]-8).tolist();modelmanifest=json.loads((H/'dense_adapter_v2_predictions/manifest.json').read_text())
for r in modelmanifest['rows']:assert sha(r['path'])==r['sha256'];inputs[r['path']]=r['sha256']
mask=H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz';inputs[str(mask)]=sha(mask)
source=[H/x for x in ['dense_source_points.py','test_dense_source_points.py','point_support_shadow.py','test_point_support_shadow.py','stereo_geometry.py','run_dense_source_memory.py','seal_dense_source_memory.py']]+[W/'docs/superpowers/specs/2026-10-06-dense-source-memory-consumer-design.md',W/'docs/superpowers/plans/2026-10-06-dense-source-memory-consumer.md']
p=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),mask=str(mask),origin=origin,shape=[161,161],resolution=.1,maxage=10,support_radius=.7,source_sha256={str(f):sha(f) for f in source},input_sha256=inputs,depth_model_admitted=False,knownpose_layer=True,complete_same_sequence_development=True,noGTlabels_orfuture_states=True,whole_research_complete=False)
(H/'dense_source_memory_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print('presealed',len(source),len(inputs))
