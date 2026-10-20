from pathlib import Path
import hashlib,json,shutil
SOURCE=Path(__file__).resolve().parent
ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
DEST=ROOT/'research/ball_wm/runs/benchmark_oracle_stateful_replan_20260928_v1'
REPORT='benchmark_oracle_stateful_replan_20260928.md'
if DEST.exists():raise RuntimeError('Refusing to overwrite archive')
if (ROOT/'docs'/REPORT).exists():raise RuntimeError('Refusing to overwrite report')
shutil.copytree(SOURCE,DEST,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
shutil.copy2(SOURCE/REPORT,ROOT/'docs'/REPORT)
checks={str(p.relative_to(DEST)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(DEST.rglob('*')) if p.is_file()}
(DEST/'archive_sha256.json').write_text(json.dumps(checks,indent=2)+'\n')
assert all(hashlib.sha256((DEST/k).read_bytes()).hexdigest()==v for k,v in checks.items())
assert (ROOT/'docs'/REPORT).read_bytes()==(SOURCE/REPORT).read_bytes()
print(json.dumps({'archive':str(DEST),'verified_files':len(checks),'report':str(ROOT/'docs'/REPORT)},ensure_ascii=False))
