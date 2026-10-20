from paths import *
import json,hashlib,subprocess,datetime
from probe_cleanup import seal_existing
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((D/'navigation_status.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
cfg=json.loads((D/'evaluation_protocol.json').read_text());assert not (D/'evaluation_receipt.json').exists()
for p in cfg['metadata_only_derivations']:
 assert sha(p['derived'])==p['derived_sha256'] and sha(p['original'])==p['original_sha256']
out=D/'e1_run';nav=json.loads((out/'protocol_at_dispatch.json').read_text())
for field in ['source_sha256','input_sha256']:
 for f,h in nav[field].items():assert sha(f)==h,f
manifest=out/'artifact_manifest_post_close.json'
if not manifest.exists():seal_existing(out,dict(execution_status=status['status'],returncode=status['child_returncode'],purpose='A actual complete run before teacher evaluation'))
def verify():
 for f,h in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==h,f
verify()
for name in ['A_evaluate_navigation.py','A_supported_memory_diagnostic.py']:subprocess.run([sys.executable,'-B',str(D/name)],check=True)
verify()
(D/'evaluation_receipt.json').write_text(json.dumps(dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),whole_run_manifest_sha256=sha(manifest),evaluation_protocol_sha256=sha(D/'evaluation_protocol.json'),result_sha256=sha(D/'e1_result.json'),support_result_sha256=sha(D/'supported_memory_result.json'),verified_before_after=True),indent=2)+'\n')
