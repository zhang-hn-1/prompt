from pathlib import Path
import json,hashlib,datetime,copy
import numpy as np
from scipy.spatial import cKDTree
from partial_cue_source_v2 import PartialCueSetSource
H=Path(__file__).resolve().parent;D=H/'offset_replication_D4_run_v1';O=H/'offset_source_coverage_diagnosis_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in manifest.items())
seed=np.load(D/'immutable_possible_set_birth.npz');end=int(seed['frame']);records=json.loads((D/'source_opportunity_records.json').read_text());fact=PartialCueSetSource(.2);cf=PartialCueSetSource(.2);checked=0
for r in records:
 i=r['frame']
 if i>end:break
 t=float(r['time_s']);packet=dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz'));rgb=dict(np.load(D/'online_inputs'/f'frame_{i:03d}.npz'));est=dict(np.load(D/'estimated_poses'/f'frame_{i:03d}.npz'));known=dict(np.load(D/'known_poses'/f'frame_{i:03d}.npz'));mask=dict(np.load(D/'masks'/f'frame_{i:03d}.npz'))
 poses={k:est[k] for k in ['left_camera_usd_world','right_camera_usd_world']};a=fact.update(packet,rgb,poses,mask,r['qualified_seed'],t,i,est['root_position_world_m'][:2],np.array([2.2,.15]),np.array([-2.2,.15]))
 saved=np.load(D/'possible_sets'/f'frame_{i:03d}.npz');assert set(a)==set(saved)
 for k in a:np.testing.assert_equal(a[k],saved[k]);checked+=1
 b=cf.update(packet,rgb,known,mask,r['qualified_seed'],t,i,est['root_position_world_m'][:2],np.array([2.2,.15]),np.array([-2.2,.15]))
assert end==20
for k in fact.model:np.testing.assert_equal(fact.model[k],cf.model[k])
np.testing.assert_equal(fact.vox,cf.vox)
np.savez_compressed(O/'factual.npz',**a);np.savez_compressed(O/'camera_pose_only_counterfactual.npz',**b)
(O/'pre_score_seal.json').write_text(json.dumps({str(f):sha(f) for f in O.glob('*.npz')},indent=2)+'\n')
truth=np.load(D/'offline_occupancy_label.npz');xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);tol=np.sqrt(2)*.05+1e-9
metrics={}
for name,p in [('factual',a),('camera_pose_only_counterfactual',b)]:
 dist=cKDTree(p['possible_xy']).query(xy)[0];raw=cKDTree(p['raw_world_voxel_centres'][:,:2]).query(xy)[0];miss=dist>tol
 metrics[name]=dict(coverage=float((~miss).mean()),total_cells=len(xy),missed_cells=int(miss.sum()),missing_xy=xy[miss].tolist(),missing_possible_distance_m=dist[miss].tolist(),max_positive_voxel_distance_m=float(raw.max()),possible_area_m2=len(p['possible_xy'])*.01,passes_existing_95_gate=bool((~miss).mean()>=.95))
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),factual_saved_fields_exact=checked,immutable_model_and_lattice_equal=True,camera_pose_only_intervention=True,offline_privileged_not_actor=True,historical_not_fresh=True,old_D4_reject_preserved=True,metrics=metrics,pose_only_explanation='supported_within_frozen_source_projection' if metrics['camera_pose_only_counterfactual']['passes_existing_95_gate'] else 'insufficient_to_repair_frozen_coverage',no_radius_scan=True,no_inference_or_actual_run=True,whole_research_complete=False,source_sha256={str(Path(__file__)):sha(__file__),str(H/'partial_cue_source_v2.py'):sha(H/'partial_cue_source_v2.py')})
(O/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='metrics'},indent=2));print({n:{k:v for k,v in m.items() if not k.startswith('missing')} for n,m in metrics.items()})
