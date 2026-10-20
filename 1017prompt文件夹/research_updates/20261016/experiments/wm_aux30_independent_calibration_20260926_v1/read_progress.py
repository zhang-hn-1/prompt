from pathlib import Path
import json
r=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1')
print((r/'status.json').read_text())
if (r/'collection_summary.json').exists():
 for row in json.loads((r/'collection_summary.json').read_text()):
  q=json.loads((r/('qc_'+row['id']+'.json')).read_text());print(row['id'],round(row['wall_seconds'],1),'QC',q['passed'],'coverage',q['coverage'],'collisions',q['collision_frames'])
print('\n'.join((r/'execution.log').read_text().splitlines()[-5:]))
