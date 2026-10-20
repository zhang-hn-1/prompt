"""Breaks caught: treating a modified/network pickle as a trusted checkpoint."""
import zipfile
import pytest
import checkpoint_loader as c

def test_static_audit_rejects_executable_global(tmp_path):
    path=tmp_path/'unsafe.pth'
    with zipfile.ZipFile(path,'w') as z:z.writestr('archive/data.pkl',b'\x80\x02cos\nsystem\n.')
    with pytest.raises(ValueError,match='unapproved'):c.inspect_globals(path)

def test_hash_mismatch_rejected_before_loading(tmp_path):
    path=tmp_path/'changed.pth';path.write_bytes(b'not the official checkpoint')
    with pytest.raises(ValueError,match='hash'):c.verify_checkpoint(path)
