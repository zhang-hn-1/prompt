import pytest
def test_archived_plan_identity_accepts_only_exact_original_and_recorded_closure():
 # Break caught: blindly skipping a changed plan or accepting an unrecorded suffix.
 import hashlib
 import verify_point_support_shadow as verifier
 original=b'plan v1\n';closed=original+b'\n\n## 00:36UTC original-scene active observation\nclosure\n'
 expected=hashlib.sha256(original).hexdigest();final=hashlib.sha256(closed).hexdigest()
 assert verifier.archived_plan_identity(closed,expected,final)==original
 with pytest.raises(ValueError):verifier.archived_plan_identity(closed+b'changed',expected,final)
 with pytest.raises(ValueError):verifier.archived_plan_identity(closed.replace(b'v1',b'v2'),expected,hashlib.sha256(closed.replace(b'v1',b'v2')).hexdigest())
