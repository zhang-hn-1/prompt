"""Development assessment of sealed positive partial surfaces; no actor writes."""
from pathlib import Path
import datetime, hashlib, json
import numpy as np
from audit_stereo import summarize
from stereo_geometry import FOCAL

H = Path(__file__).resolve().parent
D = H / 'native_flat_pose_navigation_R1_run_v1'
O = H / 'partial_metric_surface_memory_preflight_v1'
O.mkdir(exist_ok=False)
(O / 'observations').mkdir()
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest = json.loads((D / 'artifact_manifest_post_close.json').read_text())['artifact_sha256']
protocol = json.loads((H / 'native_flat_pose_navigation_protocol_v1.json').read_text())
for group in (manifest, protocol['source_sha256']):
    for f, s in group.items():
        assert sha(f) == s, f

# Selection and accumulation finish BEFORE any truth namespace is opened.
records = []
input_hashes = {}
for frame in range(0, 601, 5):
    name = f'frame_{frame:03d}.npz'
    source_path = D / 'sensor_packets' / f'source_{frame:03d}.npz'
    packet = np.load(source_path)
    pose_path = D / 'estimated_poses' / name
    mask_paths = [D / 'partial_class_masks' / f'birth_{frame:03d}_{eye}_{frame:03d}.npz' for eye in ('left', 'right')]
    count = len(packet['world'])
    selected = np.zeros(count, dtype=bool)
    classifier_present = all(p.exists() for p in mask_paths)
    if classifier_present:
        masks = [np.load(p) for p in mask_paths]
        for mask in masks:
            assert float(mask['exposure_time_s']) <= float(packet['time_s']) + 1e-8
            assert float(mask['hypothesis_birth_time_s']) <= float(packet['time_s']) + 1e-8
            assert float(mask['source_seed_time_s']) <= float(packet['time_s']) + 1e-8
        selected = masks[0]['potential_hazard_mask'].ravel()[packet['query_pixel_id']] & masks[1]['potential_hazard_mask'].ravel()[packet['partner_pixel_id']]
    observed = {key: packet[key][selected].copy() for key in ('world', 'sigma_m', 'query_pixel_id', 'partner_pixel_id')}
    assert np.isfinite(observed['world']).all()
    path = O / 'observations' / name
    np.savez_compressed(path, **observed, frame=frame, acquisition_time_s=float(packet['time_s']), provisional_occupied_surface=True, certified_free=False, unseen_volume_observed=False)
    inputs = [source_path, pose_path] + (mask_paths if classifier_present else [])
    hashes = {str(p): sha(p) for p in inputs}
    input_hashes.update(hashes)
    records.append(dict(frame=frame, time_s=float(packet['time_s']), source_points=count, selected_points=int(selected.sum()), classifier_present=classifier_present, observation=str(path), observation_sha256=sha(path), provenance_sha256=hashes))
seal = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), records=records, input_sha256=input_hashes, code_sha256=sha(__file__), plan_sha256=sha(H.parent.parent / 'docs/superpowers/plans/2026-10-08-partial-metric-surface-memory-preflight.md'), not_fresh_heldout=True, no_truth_in_selection=True, no_free_writes=True, no_actor_consumption=True)
(O / 'memory_seal_before_truth.json').write_text(json.dumps(seal, indent=2) + '\n')

errors, classes, frame_scores = [], [], []
for record in records:
    frame = record['frame']
    observation = np.load(record['observation'])
    truth = np.load(D / 'offline_truth' / f'frame_{frame:03d}.npz')
    ids = observation['query_pixel_id']
    y, x = np.divmod(ids, 640)
    z = truth['left_depth_m'].ravel()[ids]
    rays = np.column_stack(((x-319.5)/FOCAL, (y-239.5)/FOCAL, np.ones(len(ids))))
    T = truth['left_camera_usd_world']
    true_world = T[:3, 3] + (rays * z[:, None]) @ (T[:3, :3] @ np.diag([1, -1, -1])).T
    valid = np.isfinite(z) & (z > 0) & np.isfinite(true_world).all(axis=1)
    error = np.linalg.norm(observation['world'] - true_world, axis=1)
    error[~valid] = np.inf
    # This independent fixture label is used ONLY after the memory is sealed.
    obstacle = valid & (np.linalg.norm(true_world[:, :2], axis=1) <= .351) & (true_world[:, 2] > .001) & (true_world[:, 2] <= 1.001)
    errors.append(error)
    classes.append(obstacle)
    frame_scores.append(dict(frame=frame, selected_points=len(ids), world_metric=summarize(error), true_obstacle_fraction=float(obstacle.mean()) if len(ids) else None))
    print('PARTIAL_SURFACE_ASSESSED', frame, len(ids), flush=True)
all_errors, all_classes = np.concatenate(errors), np.concatenate(classes)
metric = summarize(all_errors)
precision = float(all_classes.mean()) if len(all_classes) else 0.
checks = dict(minimum_100=metric['n'] >= 100, metric_95=metric['within_0p10m'] >= .95, p90_0p10=metric['finite_p90_m'] is not None and metric['finite_p90_m'] <= .1, obstacle_precision_95=precision >= .95)
result = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), decision='PASS_DEVELOPMENT_PARTIAL_POSITIVE_SURFACES_ONLY' if all(checks.values()) else 'REJECT_PARTIAL_METRIC_SURFACE_SOURCE', checks=checks, world_metric=metric, true_obstacle_precision=precision, total_exposures=121, exposures_with_selected_points=sum(r['selected_points'] > 0 for r in records), classifier_absent_exposures=sum(not r['classifier_present'] for r in records), frames=frame_scores, not_fresh_heldout=True, not_online_memory_validation=True, no_dense_or_blind_coverage_claim=True, missing_is_unknown=True, no_free_writes=True, no_actor_change=True, old_dense_gate_reject_preserved=True, whole_research_complete=False)
(O / 'assessment.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
print(json.dumps({k:v for k,v in result.items() if k != 'frames'}, indent=2))
