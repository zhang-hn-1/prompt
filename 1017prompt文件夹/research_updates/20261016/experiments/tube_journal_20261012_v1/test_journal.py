import json
import pytest
from journal import Journal

def test_commit_before_callback(tmp_path):
 p=tmp_path/'log';j=Journal(p);j.commit(b'snapshot',tuple((.3,.1) for _ in range(25)))
 def consume(pair):assert json.loads(p.read_text().splitlines()[0])['type']=='commit'
 j.consume(consume);assert j.index==1

def test_no_overwrite(tmp_path):
 p=tmp_path/'log';Journal(p)
 with pytest.raises(FileExistsError):Journal(p)
def test_no_uncommitted_callback(tmp_path):
 j=Journal(tmp_path/'log')
 with pytest.raises(ValueError):j.consume(lambda p:None)
def test_failure_closes(tmp_path):
 j=Journal(tmp_path/'log');j.commit(b'x',tuple((0.,0.) for _ in range(25)))
 def fail(p):raise RuntimeError('stop')
 with pytest.raises(RuntimeError):j.consume(fail)
 with pytest.raises(ValueError):j.consume(lambda p:None)
def test_full_once(tmp_path):
 j=Journal(tmp_path/'log');j.commit(b'x',tuple((0.,0.) for _ in range(25)))
 for _ in range(25):j.consume(lambda p:None)
 with pytest.raises(ValueError):j.consume(lambda p:None)

def test_reentrant_dispatch_rejected(tmp_path):
 j=Journal(tmp_path/"log");j.commit(b"x",tuple((0.,0.) for _ in range(25)))
 def callback(pair):j.consume(lambda p:None)
 with pytest.raises(ValueError):j.consume(callback)
 assert j.closed
 assert sum(json.loads(line)["type"]=="dispatch_intent" for line in (tmp_path/"log").read_text().splitlines())==1
