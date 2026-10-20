"""Posttermination closure for all action/map/trajectory artifacts, no live edits."""
from paths import *
import json,hashlib,subprocess
from probe_cleanup import seal_existing
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((D/'navigation_status.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
out=D/'e1_run';nav=json.loads((out/'protocol_at_dispatch.json').read_text())
for field in ['source_sha256','input_sha256']:
    for f,h in nav[field].items():assert sha(f)==h,f
assert sha(D/'evaluate_navigation.py')==nav['source_sha256'][str(D/'evaluate_navigation.py')]
manifest=out/'artifact_manifest_post_close.json'
if not manifest.exists():seal_existing(out,dict(execution_status=status['status'],returncode=status['child_returncode'],purpose='Seal complete baseline action/map/trajectory/video evidence before teacher scoring'))
def verify():
    for f,h in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==h,f
verify();subprocess.run([sys.executable,str(D/'evaluate_navigation.py')],check=True);verify()
(D/'evaluation_receipt.json').write_text(json.dumps(dict(whole_run_manifest_sha256=sha(manifest),evaluator_sha256=sha(D/'evaluate_navigation.py'),result_sha256=sha(D/'e1_result.json'),verified_before_after=True),indent=2)+'\n')
