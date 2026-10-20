"""Administrative byte identity, no observation or feedback interpretation."""
import hashlib,json
from pathlib import Path

def verify_identity(identity_path):
    for name,expected in json.loads(Path(identity_path).read_text()).items():
        actual=hashlib.sha256(Path(name).read_bytes()).hexdigest()
        if actual!=expected:raise ValueError('frozen dependency changed: '+name)
