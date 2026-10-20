import json,hashlib
import pytest
from reverse_half_development_admission_v1 import require_development_point
def test_unverified_or_failed_development_cannot_start_validation(tmp_path):
    def write(name,j):
        p=tmp_path/name;p.write_text(json.dumps(j));return p
    p=write('reverse_half_fitted_point_v1.json',dict(decision='DEVELOPMENT_POINT_SEALED',utc='2026-10-10',provenance_sha256={}))
    with pytest.raises((ValueError,FileNotFoundError)):require_development_point(tmp_path,{'utc':'2026-10-09'})
    status=write('reverse_half_development_status_v1.json',dict(status='finished',child_returncode=1,end_utc='2026-10-09'))
    with pytest.raises((ValueError,FileNotFoundError)):require_development_point(tmp_path,{'utc':'2026-10-09'})
