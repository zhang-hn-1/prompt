import hashlib,json
import pytest

def test_identity_rejects_changed_dependency(tmp_path):
 from memory_availability_identity_v1 import verify_identity
 p=tmp_path/'dependency';p.write_bytes(b'original')
 identity=tmp_path/'identity.json';identity.write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest()}))
 verify_identity(identity)
 p.write_bytes(b'changed')
 with pytest.raises(ValueError):verify_identity(identity)

def test_feature_receipt_rejects_changed_index_before_teacher(tmp_path):
 from memory_availability_identity_v1 import verify_identity
 p=tmp_path/'feature_index';p.write_bytes(b'no teacher')
 receipt=tmp_path/'receipt.json';receipt.write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest()}))
 p.write_bytes(b'teacher contamination')
 with pytest.raises(ValueError):verify_identity(receipt)
