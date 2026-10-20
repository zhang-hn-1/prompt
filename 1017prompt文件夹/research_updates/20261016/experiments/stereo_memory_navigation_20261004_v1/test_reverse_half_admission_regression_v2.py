"""Additional branch coverage; frozen actor/helper bytes remain unchanged."""
import json,hashlib
import pytest
from reverse_half_development_admission_v1 import require_development_point
def test_valid_provenance_passes_then_real_failed_status_is_rejected(tmp_path):
    h=tmp_path;sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    def save(name,j):
        p=h/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(j));return p
    protocol=save('reverse_half_protocol_v1.json',dict(utc='2026-10-09'))
    status=save('reverse_half_development_status_v1.json',dict(status='finished',child_returncode=0))
    raw=save('raw.json',dict(raw=True))
    manifest=save('reverse_half_development_run_v1/post_close_manifest.json',dict(artifact_sha256={str(raw):sha(raw)}))
    measured=dict(actual_speed_m_s=.19)
    assessment=save('reverse_half_development_assessment_v1.json',dict(decision='PASS_POINT_ONLY_PENDING_EXECUTION_VERIFY',checks={'all':True},raw_manifest_sha256=sha(manifest),measured=measured))
    verify=save('reverse_half_development_verification_v1.json',dict(feedback_execution_ticks_exact=1600,physical_contacts=0,assessment_sha256=sha(assessment)))
    def point():return save('reverse_half_fitted_point_v1.json',dict(decision='DEVELOPMENT_POINT_SEALED',utc='2026-10-10',provenance_sha256={str(p):sha(p) for p in [protocol,status,manifest,assessment,verify]},measured=measured))
    point();assert require_development_point(h,{'utc':'2026-10-09'})['measured']==measured
    save(status.name,dict(status='runtime_failed',child_returncode=1));point()
    with pytest.raises(ValueError,match='development not closed0'):require_development_point(h,{'utc':'2026-10-09'})
