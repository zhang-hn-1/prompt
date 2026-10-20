"""No writes: sealed file identity and actual terminal state audit."""
from pathlib import Path
import json,hashlib
D=Path(__file__).resolve().parent;Q=D.parent/'pixel_surface_projection_20261013_v1';checked=0
for root in [Q,D]:
 m=json.loads((root/'artifact_manifest_post_close.json').read_text())
 for p,s in m['artifact_sha256'].items():
  assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==s,p;checked+=1
q=json.loads((Q/'qualification.json').read_text());r=json.loads((D/'result.json').read_text())
assert q['qualification_pass'] and q['fit']==0 and q['physical']==0
assert not r['source_qualified'] and r['fit']==1 and r['physical']==0
assert len(q['rows'])==len(r['rows'])==5 and all(not a['pass_gate'] for a in r['rows'])
for name in ['prepare','train_predict','score','factor_diagnostic','visuals','visuals_reviewed']:
 s=json.loads((D/(name+'_status.json')).read_text());assert s['status']=='finished' and s['child_returncode']==0 and not s['timeout']
assert json.loads((D/'independent_closure.json').read_text())['independent_review_pass']
assert json.loads((D/'budget_close.json').read_text())['unused_physical_closed']==4
print(json.dumps(dict(verified_files=checked,oracle_qualified=True,learned_source_rejected=True,fit=1,physical=0,whole_complete=False)))
