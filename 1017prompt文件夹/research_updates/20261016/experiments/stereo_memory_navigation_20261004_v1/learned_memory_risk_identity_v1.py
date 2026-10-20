"""Pre-fit immutable implementation/dataset identity; evaluated before stage payloads."""
from pathlib import Path
import json,hashlib
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify_prefit_identity(H):
 H=Path(H);p=H/'learned_memory_risk_prefit_identity_v2.json';identity=json.loads(p.read_text())
 assert identity['fit_not_executed_at_freeze'] and identity['fresh_teacher_not_inspected_at_freeze']
 for path,digest in identity['input_sha256'].items():assert sha(path)==digest,path
 return sha(p)
