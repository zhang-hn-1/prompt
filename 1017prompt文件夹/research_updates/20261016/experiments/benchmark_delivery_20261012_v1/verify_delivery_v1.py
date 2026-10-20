"""Repeatable read-only integrity check; never launches physics or rescores trials."""
from pathlib import Path
import json,hashlib
HERE=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 manifest=json.loads((HERE/'delivery_manifest_v1.json').read_text())
 for file,digest in manifest['delivery_sha256'].items():
  if sha(file)!=digest:raise ValueError('delivery identity mismatch: '+file)
 audit=json.loads((HERE/'existing_evidence_audit_v1.json').read_text());pose=json.loads((HERE/'saved_pose_diagnostic_v1.json').read_text())
 table=dict(audit['verified_sha256'])
 for file,digest in pose['inputs_sha256'].items():
  if file in table and table[file]!=digest:raise ValueError('input identity conflict: '+file)
  table[file]=digest
 for file,digest in table.items():
  if sha(file)!=digest:raise ValueError('source identity mismatch: '+file)
 if len(audit['matrix'])!=12 or len(pose['rows'])!=6:raise ValueError('condition coverage')
 for key in ['new_physics','new_policy_act','new_model_queries','new_teacher_queries','new_training']:
  if audit[key]!=0:raise ValueError('unexpected new experiment budget')
 print(json.dumps(dict(delivery_integrity=True,verified_original_files=len(table),conditions=12,pose_diagnostics=6,memory_boundary_checks=audit['memory']['verified_same_exposure_boundaries'],reliable_navigation_complete=False,learning_navigation_increment_complete=False,whole_mission_complete=False)))
if __name__=='__main__':main()
