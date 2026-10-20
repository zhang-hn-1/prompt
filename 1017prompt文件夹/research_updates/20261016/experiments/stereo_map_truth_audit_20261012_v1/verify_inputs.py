from pathlib import Path
import hashlib,json
D=Path(__file__).resolve().parent;W=D.parents[1];V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';R=V/'candidate_e2_run';m=json.loads((V/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];files=[R/'native_predictions.json',R/'states.json',V/'viewpoint_confirmation.py']
for name in ['partial_observations','frames','known_poses','maps','raw_point_memory_snapshots']:
 files.extend(sorted((R/name).glob('*')))
files.append(R/'source_predictions/frame_400.npz')
for p in files:
 assert str(p) in m,p
 assert hashlib.sha256(p.read_bytes()).hexdigest()==m[str(p)],p
result=dict(verified_source_files=len(files),existing_seal_unchanged=True,comparison='SHA256 of every evaluator-consumed saved data file against preexisting seal',truth_online=False,new_physics_runs=0)
(D/'input_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
