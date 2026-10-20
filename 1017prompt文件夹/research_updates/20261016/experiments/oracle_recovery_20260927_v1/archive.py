from pathlib import Path
import hashlib,json,shutil,subprocess,datetime
here=Path(__file__).resolve().parent
root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
dest=root/'research/ball_wm/runs/benchmark_oracle_entry_recovery_20260928_v1'
doc=root/'docs/benchmark_oracle_entry_recovery_20260928.md'
if dest.exists() or doc.exists():raise FileExistsError('refuse to replace an archive')
versions=here/'source_versions';versions.mkdir(exist_ok=True)
current=(here/'probe.py').read_text()
new='''            from source_map import SourceMapReplanner
            replan=SourceMapReplanner(case,kwargs['drive_model'])
            write(out/'map_provenance.json',replan.provenance)
'''
old='''            from benchmark.kinodynamic_planning import KinodynamicPlanRequest,plan_kinodynamic_path
            def replan(obs,goal):
                req=KinodynamicPlanRequest(occupancy_grid=runtime._inflated_grid,resolution_m=runtime._occupancy.resolution_m,origin_xy=tuple(runtime._occupancy.global_origin_xy),start_xy=tuple(obs.position_world_xy),start_yaw_rad=obs.yaw_world_rad,goal_xy=tuple(goal),node_budget=5000)
                return plan_kinodynamic_path(req,kwargs['drive_model'])
'''
empty=current.replace(new,old)
initial=empty
start=initial.index('        kwargs=dict(lookahead_m=')
end=initial.index('        policy.reset(',start)
initial=initial[:start]+'''        policy=EventOracle(protocol,lookahead_m=.5,yaw_rate_feedback_gain_s=.15,drive_model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE))
'''+initial[end:]
initial=initial.replace('''        if condition=='candidate':
            write(out/'replan.json',dict(attempted=policy.replan_attempted,seconds=policy.replan_seconds,result=policy.replan_result))
''','').replace("choices=['real','aligned','candidate']","choices=['real','aligned']")
matched={}
for label,source,run in [('diagnostic',initial,'0002_real'),('invalid_empty_map',empty,'0002_candidate'),('correct_source_map',current,'0002_candidate_source_map')]:
    digest=hashlib.sha256(source.encode()).hexdigest()
    expected=json.loads((here/run/'config.json').read_text())['source_sha256']
    matched[label]=dict(sha256=digest,expected=expected,exact_match=digest==expected)
    if digest!=expected:raise RuntimeError(f'cannot reconstruct recorded source version {label}')
    (versions/f'probe_{label}.py').write_text(source)
(here/'source_version_verification.json').write_text(json.dumps(matched,indent=2))
snapshot=here/'implementation_snapshot';snapshot.mkdir(exist_ok=True)
files=['benchmark/policies.py','benchmark/isaac_runtime.py','benchmark/runner.py','benchmark/drive_model.py','benchmark/kinodynamic_planning.py','benchmark/grid_planning.py','scripts/action_smoothing.py','scripts/run_round4_fixed_reference.py','scripts/run_tracking_acceptance.py','scripts/analyze_round8_tracking_audit.py','configs/data_contract.yaml','configs/benchmark_v1/drive_dynamics_v1.json','tests/test_round8_tracking_audit.py']
hashes={}
for rel in files:
    src=root/'research/ball_wm'/rel;target=snapshot/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target);hashes[rel]=hashlib.sha256(src.read_bytes()).hexdigest()
(here/'implementation_sha256.json').write_text(json.dumps(hashes,indent=2))
before=json.loads((here/'test_correction.json').read_text())['before_sha256']
baseline=json.loads((root/'research/ball_wm/runs/benchmark_golden_oracle_reproduced_20260922_v1/audit/source_sha256.json').read_text())['research/ball_wm/tests/test_round8_tracking_audit.py']
assert before==baseline
(here/'provenance.json').write_text(json.dumps(dict(worktree=str(root),git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),snapshot_recorded='after final execution; harness versions verified against per-run before-execution SHA',completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_test_matched_golden_snapshot=True,production_control_edited_this_turn=False,accepted_candidate=False),indent=2))
shutil.copytree(here,dest,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
shutil.copy2(here/doc.name,doc)
manifest={str(p.relative_to(dest)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dest.rglob('*')) if p.is_file()}
(dest/'archive_sha256.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(dict(archive=str(dest),report=str(doc),files=len(manifest),source_versions=matched),indent=2))
