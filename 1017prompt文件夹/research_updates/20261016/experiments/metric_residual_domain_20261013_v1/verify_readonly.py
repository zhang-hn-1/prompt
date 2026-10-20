from pathlib import Path
import json,hashlib
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
m=json.loads((D/'artifact_manifest_post_close.json').read_text())
for p,s in m['artifact_sha256'].items():assert sha(p)==s,p
r=json.loads((D/'result.json').read_text())
for key in ['input_sha256','output_sha256','source_sha256']:
 for p,s in r[key].items():assert sha(p)==s,p
assert not r['qualified'] and not r['optimistic_qualified'] and r['fit']==r['physical']==0
assert all(not a['qualification_pass'] and not a['optimistic_semantic_pass'] for a in r['rows'])
s=json.loads((D/'qualify_status.json').read_text());assert s['status']=='finished' and s['child_returncode']==0
assert json.loads((D/'independent_closure.json').read_text())['independent_review_pass']
print(json.dumps(dict(sealed_files=len(m['artifact_sha256']),pairs=905,rejected=True,fit=0,physical=0,whole_complete=False)))
