"""Archive the frozen candidate and its final, comparable raw runs."""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path


LOCAL = Path('/home/an/Documents/ChatGPT/球型机器人导航')
EXPERIMENT = LOCAL / 'experiments/oracle_feasible_route_20260929_v9'
REPO = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
ARCHIVE = REPO / 'research/ball_wm/runs/benchmark_oracle_measured_time_hybrid_20260929_v9'
REPORT = 'benchmark_oracle_measured_time_hybrid_navigation_20260929.md'


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main() -> None:
    if ARCHIVE.exists():
        raise RuntimeError(f'archive exists: {ARCHIVE}')
    ARCHIVE.mkdir(parents=True)

    dirs = [EXPERIMENT / 'hybrid']
    for pattern in ('navigation_hybrid_hard_*', 'navigation_hybrid_golden_*',
                    'navigation_physical_baseline_*'):
        dirs.extend(sorted(p for p in EXPERIMENT.glob(pattern) if p.is_dir()))
    for src in dirs:
        shutil.copytree(src, ARCHIVE / src.name,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))

    base = EXPERIMENT.parent / 'oracle_collision_fix_20260929_v8/pid'
    if not base.is_dir():
        raise RuntimeError(f'missing original baseline worker: {base}')
    shutil.copytree(base, ARCHIVE / 'baseline_worker',
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))

    names = {
        'full_comparison.json', 'hard0_e1_trajectories.png', 'verify_full.py',
        'plot_comparison.py', 'test_hybrid_planning.py', 'test_gated_speed.py',
        'test_speed_schedule.py', 'offline_results.json', 'assess.py',
        'assessment.json', 'selected_candidate.json', 'run_hybrid_hard.py',
        'run_hybrid_golden.py', 'run_physical_baseline.py', 'archive_selected.py',
    }
    prefixes = ('hybrid_hard_', 'hybrid_golden_', 'physical_baseline_')
    names.update(p.name for p in EXPERIMENT.iterdir()
                 if p.is_file() and p.name.startswith(prefixes))
    for name in sorted(names):
        src = EXPERIMENT / name
        if not src.is_file():
            raise RuntimeError(f'missing file: {src}')
        shutil.copy2(src, ARCHIVE / name)

    (ARCHIVE / 'README.md').write_text(
        '# Measured-time hybrid Oracle, 2026-09-29\n\n'
        'Frozen experimental candidate: `hybrid/`. The production/default Oracle '
        'is unchanged. Under the original 35-second limit and physical-contact v2 '
        'adjudication, hard_0 changed from 3/7 to 5/7 successes; golden E0/E1/E2 '
        'remained 3/3. E1 0002 and 0004 still timed out.\n\n'
        '`navigation_hybrid_hard_*`, `navigation_hybrid_golden_*`, and '
        '`navigation_physical_baseline_*` are the raw comparable runs. '
        '`*_command.json`, `*_status.json`, source identity JSON, and source '
        'snapshots preserve the execution evidence. `full_comparison.json` and '
        '`verify_full.py` summarize and verify it. Re-execution requires the '
        'project Isaac Sim environment and scene assets; choose fresh empty '
        'output directories rather than overwriting these frozen runs.\n\n'
        'The full report is `../../../../docs/' + REPORT + '`. Rejected and '
        'intermediate trials remain in the local experiment directory.\n',
        encoding='utf-8',
    )
    report_target = REPO / 'docs' / REPORT
    report_target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(LOCAL / 'reports' / REPORT, report_target)

    manifest = {
        str(path.relative_to(ARCHIVE)): digest(path)
        for path in sorted(ARCHIVE.rglob('*')) if path.is_file()
    }
    (ARCHIVE / 'archive_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    assert all(digest(ARCHIVE / rel) == sha for rel, sha in manifest.items())
    assert digest(report_target) == digest(LOCAL / 'reports' / REPORT)
    print(json.dumps({'archive': str(ARCHIVE), 'file_count': len(manifest),
                      'raw_run_count': len(dirs) - 1,
                      'report': str(report_target)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
