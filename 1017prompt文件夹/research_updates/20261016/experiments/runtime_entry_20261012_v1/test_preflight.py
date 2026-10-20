import hashlib
import pytest
from preflight import file_issues,decision

def test_matching_file(tmp_path):
 p=tmp_path/'f';p.write_bytes(b'known')
 assert file_issues({str(p):hashlib.sha256(b'known').hexdigest()})==[]

def test_changed_file(tmp_path):
 p=tmp_path/'f';p.write_bytes(b'changed')
 assert file_issues({str(p):hashlib.sha256(b'known').hexdigest()})[0]['reason']=='sha_mismatch'

def test_missing_file(tmp_path):
 assert file_issues({str(tmp_path/'missing'):'0'*64})[0]['reason']=='missing_file'

@pytest.mark.parametrize('key',['identity_match','assets_present','modules_discoverable','gpu_visible'])
def test_each_missing_requirement_not_ready(key):
 flags=dict(identity_match=True,assets_present=True,modules_discoverable=True,gpu_visible=True);flags[key]=False
 d=decision(**flags)
 assert not d['runtime_preflight_ready'] and not d['physics_launch_admitted']

def test_ready_is_not_admission():
 d=decision(identity_match=True,assets_present=True,modules_discoverable=True,gpu_visible=True)
 assert d['runtime_preflight_ready'] and not d['physics_launch_admitted']
