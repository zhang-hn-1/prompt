from pathlib import Path
import json,hashlib,shutil
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
SOURCE=HERE/'oracle_time_margin_20260928_v1'
DEST=ROOT/'research/ball_wm/runs/benchmark_oracle_time_margin_20260928_v1'
REPORT='benchmark_oracle_time_margin_20260928.md'
if DEST.exists() or (ROOT/'docs'/REPORT).exists():raise RuntimeError('Refusing overwrite')
shutil.copytree(SOURCE,DEST,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
shutil.copy2(HERE.parent/'reports'/REPORT,DEST/REPORT)
shutil.copy2(HERE.parent/'reports'/REPORT,ROOT/'docs'/REPORT)
hashes={str(p.relative_to(DEST)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(DEST.rglob('*')) if p.is_file()}
(DEST/'archive_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
assert all(hashlib.sha256((DEST/k).read_bytes()).hexdigest()==v for k,v in hashes.items())
print(json.dumps(dict(verified_files=len(hashes),report=str(ROOT/'docs'/REPORT),archive=str(DEST)),ensure_ascii=False))
