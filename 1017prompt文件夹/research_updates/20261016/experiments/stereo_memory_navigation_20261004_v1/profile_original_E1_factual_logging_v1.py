"""One bounded archive serialization profile; no simulator/control mutation."""
from pathlib import Path
import json,time,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'original_E1_direct_memory_navigation_run_v1';O=H/'original_E1_logging_profile_v1';O.mkdir(exist_ok=False);rows=json.loads((D/'policy_records.json').read_text());N=len(rows);profiles=[]
for n in [N//4,N//2,N]:
 begin=time.monotonic();data=json.dumps(rows[:n],indent=2);serialize=time.monotonic()-begin;begin=time.monotonic();(O/f'full_prefix_{n:03d}.json').write_text(data+'\n');write=time.monotonic()-begin;profiles.append(dict(n=n,bytes=len(data.encode()),json_serialize_wall_s=serialize,write_wall_s=write,one_reconstruction_only=True))
begin=time.monotonic()
with (O/'factual_rows_compact_journal.ndjson').open('x') as f:
 for i,row in enumerate(rows):f.write(json.dumps(dict(frame=i,revision=1,row=row),separators=(',',':'))+'\n')
compact_wall=time.monotonic()-begin;rebuilt=[json.loads(line)['row'] for line in (O/'factual_rows_compact_journal.ndjson').read_text().splitlines()];assert rebuilt==rows
# Original timers end before the AFTER-execute full-list write. No claim they
# measure total parent runtime or fully isolate serialization causality.
wall=json.loads((H/'original_E1_direct_memory_navigation_stage_status_v1.json').read_text())['wall_s'];actions=sum(r['end_to_end_wall_s'] for r in rows)
a=dict(decision='ARCHIVED_LOG_SERIALIZATION_PROFILE_ONLY',rows=N,full_prefix_profiles=profiles,compact_journal_once_wall_s=compact_wall,compact_journal_bytes=(O/'factual_rows_compact_journal.ndjson').stat().st_size,row_reconstruction_exact=True,stage_wall_s=wall,sum_recorded_action_wall_s=actions,remaining_wall_includes_reset_shutdown_AND_unmeasured_postwrite_s=wall-actions,actual_driver_writes_full_list_twice_per_action=True,postexecute_full_list_write_outside_recorded_timer=True,not_runtime_speed_AB=True,no_actual_code_or_actions_changed=True,no_full60_rerun=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
