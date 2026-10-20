from pathlib import Path
import json,pytest
from policy_row_journal_v1 import PolicyRowJournal,read_journal

def row(i):return dict(frame=i,time_s=i*.1,request=[0.]*10,source_frame=0,source_time_s=0.)
def test_exact_completed_rows_and_pending_excluded(tmp_path):
 p=tmp_path/'rows.ndjson';j=PolicyRowJournal(p);a=row(0);j.planned(a);a['end_to_end_wall_s']=.2;j.executed(a);j.planned(row(1));j.close();r=read_journal(p);assert r['completed']==[a] and r['pending']==[1]
def test_planned_snapshot_not_mutated_and_request_change_rejected(tmp_path):
 p=tmp_path/'rows.ndjson';j=PolicyRowJournal(p);a=row(0);j.planned(a);a['request'][0]=1.
 with pytest.raises(ValueError):j.executed(a)
 j.close();assert json.loads(p.read_text().splitlines()[0])['row']['request'][0]==0.
def test_partial_last_line_not_completed(tmp_path):
 p=tmp_path/'rows.ndjson';j=PolicyRowJournal(p);a=row(0);j.planned(a);j.executed(a);j.close()
 with p.open('a') as f:f.write('{broken')
 r=read_journal(p);assert r['completed']==[a] and r['incomplete_tail_ignored']
def test_corrupt_completed_nonfinal_line_rejected(tmp_path):
 p=tmp_path/'rows.ndjson';p.write_text('{broken\n{}\n')
 with pytest.raises(ValueError):read_journal(p)
def test_duplicate_or_nonconsecutive_phase_rejected(tmp_path):
 j=PolicyRowJournal(tmp_path/'rows.ndjson')
 with pytest.raises(ValueError):j.planned(row(1))
 j.planned(row(0))
 with pytest.raises(ValueError):j.planned(row(0))
 j.executed(row(0))
 with pytest.raises(ValueError):j.executed(row(0))
 j.close()

def test_failed_recovery_records_failure_without_completion(tmp_path):
 from policy_row_journal_v1 import recover_to_files
 p=tmp_path/'rows.ndjson';p.write_text('{broken\n')
 result=recover_to_files(p,tmp_path/'policy_records.json',tmp_path/'recovery.json')
 assert not result['valid'] and not (tmp_path/'policy_records.json').exists()
 assert not json.loads((tmp_path/'recovery.json').read_text())['valid']
