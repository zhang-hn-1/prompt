"""Explicitly separate inherited benchmark metadata from known-command sensor probe."""
import json
from pathlib import Path
def export_task_identity(path,episode):
 result=dict(actual_task='native_current_motion_readonly_preknown60s',actual_episode=episode.to_dict(),inherited_classification_applicable=False,inherited_oracle_path_applicable=False,SPL_applicable=False,not_navigation=True,not_original_E0_E1_E2=True,preknown_commands_not_feedback=True)
 Path(path).write_text(json.dumps(result,indent=2)+'\n')
