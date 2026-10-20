import json
from pathlib import Path
import pytest
from audit_current_boundary_forward_only_v1 import select_rebuild,H

def test_actual_standby_has_no_planner_fields_but_semantic_event_is_120():
 rows=json.loads((H/'reverse_main_navigation_e1_b0_run_v1/policy_records.json').read_text())
 assert not rows[0]['planner_act_called'] and 'local_plan_created' not in rows[0]
 assert select_rebuild(rows)==120

def test_missing_field_on_real_planner_call_is_invalid():
 with pytest.raises((AssertionError,KeyError)):
  select_rebuild([dict(planner_act_called=True),dict(planner_act_called=True,local_plan_created=True),dict(planner_act_called=True,local_plan_created=True)])
