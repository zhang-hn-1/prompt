"""Read-only file identity verification; no GPU, physics, fitting or file writes."""
from common import *
seal=json.loads((D/'prediction_seal.json').read_text());counts={}
for section in ['inputs','outputs','sources']:
 for p,expected in seal[section].items():assert sha(p)==expected,p
 counts[section]=len(seal[section])
assert len(seal['rows'])==5 and all(r['epochs']==181 and r['original_packet_and_surface_exact']==181 and r['original_maps_exact']==900 for r in seal['rows'])
status=json.loads((D/'generation_recovery_status.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
r=json.loads((D/'result.json').read_text());assert r['source_pass']==all(all(v['gates'].values()) for v in r['groups']);assert r['physical_used']==0 and r['fit']==0
p=D/'artifact_manifest_post_close.json'
if p.exists():
 manifest=json.loads(p.read_text())
 for f,digest in manifest['artifact_sha256'].items():assert sha(f)==digest,f
 counts['whole_stage_files']=len(manifest['artifact_sha256'])
print(json.dumps(dict(identity_pass=True,counts=counts,source_pass=r['source_pass'],physical=0,fit=0,navigation_success_claim=False,scope='artifact identity and decision aggregation; raw scientific recomputation documented in independent_closure.json'),indent=2))
