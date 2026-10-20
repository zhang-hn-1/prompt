import json,sys
import pytest
import supervise

def fixture(tmp_path,monkeypatch):
 groups=[dict(group=f'g{i:02d}') for i in range(30)];(tmp_path/'bound_schedule_v1.json').write_text(json.dumps(dict(groups=groups)))
 monkeypatch.setattr(supervise,'D',tmp_path);monkeypatch.setattr(supervise,'validate_admission',lambda p:{});monkeypatch.setattr(sys,'argv',['supervise','--admission','fake'])
 return tmp_path/'collection_status_v1.json'
def test_dispatch_exception_keeps_reserved_attempt(tmp_path,monkeypatch):
 path=fixture(tmp_path,monkeypatch)
 def fail(*a,**kw):raise RuntimeError('launch failed')
 monkeypatch.setattr(supervise,'run_durable',fail)
 with pytest.raises(RuntimeError):supervise.main()
 d=json.loads(path.read_text());assert d['used']==1 and d['unattempted']==29 and d['status']=='parent_runtime_failed' and d['groups'][0]['group']=='g00'
def test_existing_ledger_no_dispatch(tmp_path,monkeypatch):
 path=fixture(tmp_path,monkeypatch);path.write_text('{}')
 monkeypatch.setattr(supervise,'run_durable',lambda *a,**kw:pytest.fail('must not run'))
 with pytest.raises(FileExistsError):supervise.main()
