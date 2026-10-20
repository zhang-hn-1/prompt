"""Repackage one historical sealed pilot in tmp solely to check importer, not fresh data."""
import json,hashlib,os
from pathlib import Path
import pytest
from import_group import read_group
D=Path(__file__).resolve().parent;P=D.parent/'tube_live_pilot_20261012_v1'
def fixture(tmp_path):
 run=tmp_path/'run';run.mkdir()
 for f in (P/'run_v1').iterdir():
  if f.is_file():os.symlink(f,run/f.name)
 status=tmp_path/'status.json';status.write_bytes((P/'run_status_v1.json').read_bytes())
 group=dict(group='fixture_excluded',split='qualification_only',runtime_binding=dict(episode=json.loads((run/'task_episode.json').read_text())),window_templates=['stop']*70)
 (run/'group_binding.json').write_text(json.dumps(group))
 return run,status,group

def seal(tmp_path,run,status):
 p=tmp_path/'seal.json';p.write_text(json.dumps(dict(artifact_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in list(run.iterdir())+[status]})));return p

def test_actual_pilot_records(tmp_path):
 run,status,group=fixture(tmp_path);r=read_group(run,status,seal(tmp_path,run,status),group);assert r['scorable']==70 and r['programs']==['stop']
def test_incomplete_journal_rejected(tmp_path):
 run,status,group=fixture(tmp_path);p=run/'window_000.jsonl';rows=[json.loads(l) for l in p.read_text().splitlines()];p.unlink();p.write_text('\n'.join(json.dumps(r) for r in rows if r['type'] not in ('anchor_source_receipt','block_dispatch_intent'))+'\n')
 with pytest.raises(ValueError):read_group(run,status,seal(tmp_path,run,status),group)
def test_initial_supervision_rejected(tmp_path):
 run,status,group=fixture(tmp_path);p=run/'window_000.jsonl';rows=[json.loads(l) for l in p.read_text().splitlines()];next(r for r in rows if r['type']=='initial_supervision')['world_xy']=[100.,100.];p.unlink();p.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
 with pytest.raises(ValueError):read_group(run,status,seal(tmp_path,run,status),group)

def replace_json(path,mutate):
 data=json.loads(path.read_text());mutate(data);path.unlink();path.write_text(json.dumps(data))
def test_native_timestamp_rejected(tmp_path):
 run,status,group=fixture(tmp_path);replace_json(run/'native_reads.json',lambda r:r[0].update(sensor_begin_stamp_s=r[0]['sensor_begin_stamp_s']+10))
 with pytest.raises(ValueError):read_group(run,status,seal(tmp_path,run,status),group)
def test_raw_contact_rejected(tmp_path):
 run,status,group=fixture(tmp_path);replace_json(run/'contact_samples.json',lambda r:r[0].update(contact=True))
 with pytest.raises(ValueError):read_group(run,status,seal(tmp_path,run,status),group)
def test_nonfinite_camera_rejected(tmp_path):
 run,status,group=fixture(tmp_path);p=run/'window_000.jsonl';rows=[json.loads(l) for l in p.read_text().splitlines()];rows[0]['receipt']['left_render_time_s']=float('nan');p.unlink();p.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
 with pytest.raises(ValueError):read_group(run,status,seal(tmp_path,run,status),group)
