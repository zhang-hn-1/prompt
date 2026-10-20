from pathlib import Path
import sys,json
D=Path(__file__).resolve().parent;W=D.parents[1];V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';sys.path[:0]=[str(V),str(W/'experiments/stereo_memory_navigation_20261004_v1'),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
import audit_actual
audit_actual.D=D
for ep in ['e0','e1']:
 status=D/f'data_{ep}_status.json'
 if not status.exists() or json.loads(status.read_text())['status']!='finished':continue
 if (D/f'data_{ep}_raw_audit.json').exists():
  assert json.loads((D/f'data_{ep}_raw_audit.json').read_text())['pass_audit'];continue
 r=audit_actual.audit('data_'+ep);(D/f'data_{ep}_raw_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(ep,r['pass_audit'],flush=True)
