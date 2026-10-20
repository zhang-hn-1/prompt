from pathlib import Path
import hashlib,json,shutil
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
REPORT='benchmark_oracle_recovery_and_reference_reserve_20260928.md'
report_source=HERE.parent/'reports'/REPORT
names=['oracle_same_direction_recovery_20260928_v1','oracle_handoff_presteer_20260928_v1','oracle_curvature_reserve_20260928_v1']
for name in names:
    if (ROOT/'research/ball_wm/runs'/('benchmark_'+name)).exists():raise RuntimeError('refuse overwrite')
if (ROOT/'docs'/REPORT).exists():raise RuntimeError('refuse report overwrite')
results=[]
for name in names:
    source=HERE/name;dest=ROOT/'research/ball_wm/runs'/('benchmark_'+name)
    # Final post-run identity; keep C pre_run_identity and its test-only diff intact.
    paths=list(source.glob('*.py'))
    identities={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    for rel in ['benchmark/policies.py','benchmark/drive_model.py','benchmark/kinodynamic_planning.py','benchmark/isaac_runtime.py','scripts/action_smoothing.py','scripts/bev_ground_truth.py','scripts/analyze_round8_tracking_audit.py','scripts/run_tracking_acceptance.py','scripts/run_round4_fixed_reference.py','training/action_protocol.py','configs/data_contract.yaml','configs/benchmark_v1/drive_dynamics_v1.json']:
        original=ROOT/'research/ball_wm'/rel;raw=original.read_bytes();p=source/'source_snapshot'/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw);identities[str(original)]=hashlib.sha256(raw).hexdigest()
    (source/'source_identity.json').write_text(json.dumps({'scope':'final post-run snapshot, not execution-time attestation; C has separate pre-run manifest','sha256':identities},indent=2)+'\n')
    shutil.copytree(source,dest,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copy2(report_source,dest/REPORT)
    shutil.copy2(HERE/'oracle_recovery_20260928_evidence_validation.json',dest/'evidence_validation.json')
    hashes={str(p.relative_to(dest)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dest.rglob('*')) if p.is_file()}
    (dest/'archive_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    assert all(hashlib.sha256((dest/k).read_bytes()).hexdigest()==v for k,v in hashes.items())
    results.append({'archive':str(dest),'verified_files':len(hashes)})
shutil.copy2(report_source,ROOT/'docs'/REPORT)
assert report_source.read_bytes()==(ROOT/'docs'/REPORT).read_bytes()
print(json.dumps({'archives':results,'report':str(ROOT/'docs'/REPORT)},ensure_ascii=False,indent=2))
