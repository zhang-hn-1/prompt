from native_probe_identity import export_task_identity
import json
class Episode:
 def to_dict(self):return dict(difficulty='E0',classification_evidence={'fixture':'all_free_open_plane'},oracle_path_length_m=8.7,scene_uri='${SCENE_ROOT}/single_cylinder.usda')
def test_cylinder_probe_does_not_claim_inherited_classification(tmp_path):
 path=tmp_path/'task_episode.json';export_task_identity(path,Episode());result=json.loads(path.read_text())
 assert result['actual_episode']['scene_uri'].endswith('single_cylinder.usda')
 assert result['inherited_classification_applicable'] is False and result['inherited_oracle_path_applicable'] is False
 assert result['not_navigation'] is True and result['not_original_E0_E1_E2'] is True
 assert result['actual_task']=='native_current_motion_readonly_preknown60s'
