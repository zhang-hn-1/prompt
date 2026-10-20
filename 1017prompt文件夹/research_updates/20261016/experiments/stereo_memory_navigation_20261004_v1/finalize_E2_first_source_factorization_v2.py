from pathlib import Path
import json,ast,hashlib,numpy as np
H=Path(__file__).resolve().parent;O=H/'E2_first_source_RGB_pose_factorization_v1';log=H/'E2_first_source_RGB_pose_factorization_v1.log';rows=[ast.literal_eval(s) for s in log.read_text().splitlines() if s.startswith("{'image_stream'")];assert len(rows)==4
cfg=json.loads((H/'E2_first_source_RGB_pose_factorization_protocol_v1.json').read_text());inputs={};sealed={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('*.npz')};assert len(sealed)==4
for r in rows:
 D=H/cfg['streams'][r['image_stream']];PD=H/cfg['streams'][r['pose_stream']];x=np.load(O/(r['image_stream']+'__'+r['pose_stream']+'.npz'));assert len(x['world'])==r['selected_surface_points']
 if r['image_stream']==r['pose_stream']:
  old=np.load(D/'partial_observations/frame_005.npz')
  for k in ('world','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(x[k],old[k])
 for p in (D/'source_inputs/frame_005.npz',D/'source_predictions/frame_005.npz',PD/'known_poses/frame_005.npz'):inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
r=dict(protocol=cfg,rows=rows,inputs_sha256=inputs,partial_predictions_unchanged_sha256=sealed,actual_diagonal_exact=True,teacher_depth_read=False,new_inference=False,new_physics=False,not_online_validation=True,not_navigation_success=True,old_source_rejections_preserved=True,conclusion='at .5s the 34-versus0 selected hazard difference follows RGB/cached stereo prediction, not pose swap; whole later collision cause unisolated',v1_failure='numpy int64 cells not JSON serializable after all4 numerical cases completed; no numerical rerun',repair='parse exact printed Python integer rows and validate original NPZ diagonals; no prediction/filter/threshold change')
(O/'assessment_v2.json').write_text(json.dumps(r,indent=2)+'\n');assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in sealed.items());(O/'artifact_manifest_post_finalization_v2.json').write_text(json.dumps(dict(artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.rglob('*') if p.is_file()}),indent=2)+'\n');print(r['conclusion'])
